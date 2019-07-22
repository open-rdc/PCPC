#include "PreCalculatedPreviewControl.h"

void PreCalculatedPreviewControl::interpolation_zmp_trajectory()
{
	refzmp_list.clear();
	preview_num = 0;

	// Add Virtual Control Point
	Eigen::Vector4d virtual_control_point = foot_step_list.back();
	virtual_control_point[0] += preview_delay;
	foot_step_list.push_back(virtual_control_point);

	size_t foot_step_count = 1;	
	size_t size = foot_step_list.size();
	foot_step_num = (foot_step_list[size-2](0)+stop_time)/dt;
	preview_step_num = (foot_step_list[size-2](0))/dt - 1; 

	for(int t=0;t<(foot_step_list[size-1](0)/dt);t++){
		float temp_time = t*dt;
		if(foot_step_list[foot_step_count](0) < temp_time) foot_step_count++;
		refzmp_list.push_back(Eigen::Vector2d(foot_step_list[foot_step_count-1](1), foot_step_list[foot_step_count-1](2)));
	}
}

// 多項式係数の読み込み
void PreCalculatedPreviewControl::load_coefficient()
{
	FILE *fp;
	double temp_coef=0.f;
	int para=0, index=0;

	// velocity
	if((fp=fopen("coefficient/velocity.txt","r")) != NULL){
		while(fscanf(fp,"%lf\n", &temp_coef) != EOF){
			corr[para][index] = temp_coef;
			index++;
		}
		para++; index=0;
	}

	// disp
	if((fp=fopen("coefficient/disp.txt","r")) != NULL){
		while(fscanf(fp,"%lf\n", &temp_coef) != EOF){
			corr[para][index] = temp_coef;
			index++;
		}
		para++;
	}

	// target
	if((fp=fopen("coefficient/target.txt","r")) != NULL){
		while(fscanf(fp,"%lf\n", &temp_coef) != EOF){
			corr[para][index] = temp_coef;
			index++;
		}
		para++;
	}

	// last
	if((fp=fopen("coefficient/last.txt","r")) != NULL){
		while(fscanf(fp,"%lf\n", &temp_coef) != EOF){
			corr[para][index] = temp_coef;
			index++;
		}
		para++;
	}
}

// 支持脚切り替え時点での入力パラメータの更新
void PreCalculatedPreviewControl::update_input()
{
	if(walk_status < StopWalking){
		foot_y = foot_width * ((walk_status == StartWalking) ? 0 : ((support_leg == RightLegSup) ? LeftLegSup : RightLegSup));
		target_d[0] = target[0] - ref_zmp_gc[0] - ref_zmp[0];
		target_d[1] = target[1] - ref_zmp_gc[1] - ref_zmp[1] - foot_y;
		cog[0] = com_pos[0] - ref_zmp_gc[0] - ref_zmp[0];
		cog[1] = com_pos[1] - ref_zmp_gc[1] - ref_zmp[1] - foot_y;
	}else{
		foot_y = 0;
	}
	
	std::cout << ref_zmp[0] << " " << ref_zmp[1] << std::endl;

	if(walk_status == Walking && fabs(target_d[0]) < 0.01f && fabs(target_d[1]) < 0.01f){
		walk_status = StopWalking;
		end_time = preview_num + sampling_num_half_cycle * 2;
		last_step_start_time = preview_num;
	}else if(walk_status == StopWalking){
		walk_status = StopWalkingLastStep;
	}

	//foot_planner->SetTargetPos(target_d[0], target_d[1], 0.f, support_leg, walk_status);
	
	if(walk_status < StopWalkingLastStep){
		disp[0] = com_pos[0] - ref_zmp_gc[0] - ref_zmp[0];
		disp[1] = com_pos[0] - ref_zmp_gc[1] - ref_zmp[1] - foot_y;
		dir = (support_leg == RightLegSup) ? LeftLegSup : RightLegSup;
		prev_ref_zmp[0] = ref_zmp[0];
		prev_ref_zmp[1] = ref_zmp[1];
		// Foot Step Plannerから結果を抜き出す
		double offset = 0.f;
		if(index == 1) offset = foot_width * ((support_leg == RightLegSup) ? LeftLegSup : RightLegSup);
		else if(foot_step_list.size()-2 <= index ) offset = 0;
		ref_zmp[0] = foot_step_list[index][1]-foot_step_list[index-1][1];
		ref_zmp[1] = foot_step_list[index][2]-foot_step_list[index-1][2];
		ref_zmp_gc[0] += prev_ref_zmp[0];
		ref_zmp_gc[1] += (prev_ref_zmp[1]+foot_y);
		step[0] = (walk_status == Walking) ? ref_zmp[0] : 0;
		step[1] = (walk_status == Walking) ? ref_zmp[1] - foot_y : 0;
		index++;
	}
}

#if 0
// 重心軌道を計算するための多項式係数の計算
void PreCalulatedPreviewControl::calcCoefficient_2_Coefficient(walk_status state, int offset_coef, double &target_corr[12])
{
	for(int index=0;i<6;index++)
	{
		target_corr[index] 		= corr[0][index+offset_coef*12]*vel[0] + corr[1][index+offset_coef*12]*disp[0] + corr[2][index+offset_coef*12]*
		target_corr[index+6]	=
	}
}
#endif

// 重心軌道を計算するための多項式の導出
// 複数の歩行フェーズにより構成
void PreCalculatedPreviewControl::calcCoefficient()
{
	// 一歩分の軌道を計算するための入力値の計算
	update_input();
#if 0
	switch(walk_status)
	{
		case Start:
				calcCoefficient_2_Coefficient(Walking, 0, target_corr); break;
		case Walking:
			if(fabs(target_dx) >= 0.18){
				calcCoefficient_2_Coefficient(Walking, 1, target_corr);
			}else if(fabs(target_dx) >= 0.12){
				calcCoefficient_2_Coefficient(Walking, 2, target_corr);
			}else if(fabs(target_dx) >= 0.06){
				calcCoefficient_2_Coefficient(Walking, 3, target_corr);
			}else{
				calcCoefficient_2_Coefficient(Walking, 4, target_corr);
			}
			break;
		case StopWalking:
			if((total_time - last_step_start_time) < sampling_num_half_cycle/2)
				calcCoefficient_2_Coefficient(StopWalking, 5, target_corr);
			break;
		case:
			calcCoefficient_2_Coefficient(StopWalking, 6, target_corr); break;
	}
#endif
	int phase = 0;
	if(walk_status == StartWalking){
		for(int index=0;index<6;index++)
			target_corr[index] = corr[0][index]*com_vel[0] + corr[1][index]*disp[0] + corr[3][index];
		for(int index=6;index<12;index++)
			target_corr[index] = corr[0][index]*com_vel[1] + corr[1][index]*disp[1] + corr[3][index];
	}else if(walk_status == Walking && fabs(target_d[0]) >= 0.18){
		for(int index=0;index<6;index++)
			target_corr[index] = corr[0][index+12]*com_vel[0] + corr[1][index+12]*disp[0] + corr[3][index+12];
		for(int index=6;index<12;index++)
			target_corr[index] = corr[0][index+12]*com_vel[1] + corr[1][index+12]*disp[1] + corr[2][index+12]*step[1] + corr[3][index+12]*dir;
	}else if(walk_status == Walking && fabs(target_d[0]) >= 0.12){
		for(int index=0;index<6;index++)
			target_corr[index] = corr[0][index+12*2]*com_vel[0] + corr[1][index+12*2]*disp[0] + corr[2][index+12*2]*target[0] + corr[3][index+12*2]*dir;
		for(int index=6;index<12;index++)
			target_corr[index] = corr[0][index+12*2]*com_vel[1] + corr[1][index+12*2]*disp[1] + corr[2][index+12*2]*step[1] + corr[3][index+12*2]*dir;
	}else if(walk_status == Walking && fabs(target_d[0]) >= 0.06){
		for(int index=0;index<6;index++)
			target_corr[index] = corr[0][index+12*3]*com_vel[0] + corr[1][index+12*3]*disp[0] + corr[2][index+12*3]*target[0] + corr[3][index+12*3]*dir;
		for(int index=6;index<12;index++)
			target_corr[index] = corr[0][index+12*3]*com_vel[1] + corr[1][index+12*3]*disp[1] + corr[2][index+12*3]*target[1] + corr[3][index+12*3]*dir;
	}else if(walk_status == Walking){
		for(int index=0;index<6;index++)
			target_corr[index] = corr[0][index+12*4]*com_vel[0] + corr[1][index+12*4]*disp[0] + corr[2][index+12*4]*target[0] + corr[3][index+12*4]*dir;
		for(int index=6;index<12;index++)
			target_corr[index] = corr[0][index+12*4]*com_vel[1] + corr[1][index+12*4]*disp[1] + corr[2][index+12*4]*target[1] + corr[3][index+12*4]*dir;
	}else if((walk_status == StopWalking) && ((preview_num - last_step_start_time) < sampling_num_half_cycle/2)){
		for(int index=0;index<6;index++)
			target_corr[index] = corr[0][index+12*5]*com_vel[0] + corr[1][index+12*5]*disp[0] + corr[2][index+12*5]*target[0] + corr[3][index+12*5]*dir;
		for(int index=6;index<12;index++)
			target_corr[index] = corr[0][index+12*5]*com_vel[1] + corr[1][index+12*5]*disp[1] + corr[2][index+12*5]*step[1] + corr[3][index+12*5]*dir;
	}else{
		for(int index=0;index<6;index++)
			target_corr[index] = corr[0][index+12*6]*com_vel[0] + corr[1][index+12*6]*disp[0] + corr[2][index+12*6]*target[0] + corr[3][index+12*6]*dir;
		for(int index=6;index<12;index++)
			target_corr[index] = corr[0][index+12*6]*com_vel[1] + corr[1][index+12*6]*disp[1] + corr[2][index+12*6]*step[1] + corr[3][index+12*6]*dir;
	}
	target_corr[0] += ref_zmp_gc[0];
	target_corr[1] += ref_zmp_gc[1];
	if(walk_status < StopWalking){
		com_vel[0] = com_vel[1] = 0.f;
		for(int i=0;i<6;i++){
			com_vel[0] += i * target_corr[i] * std::pow(gait_half_cycle, i-1);
			com_vel[1] += i * target_corr[i] * std::pow(gait_half_cycle, i-1);
		}
	}
	if(walk_status == StartWalking) walk_status = Walking;
	support_leg = (support_leg == RightLegSup) ? LeftLegSup : RightLegSup ;
}

// 重心位置・速度・加速度をアップデート
void PreCalculatedPreviewControl::calc_xk(Eigen::Vector2d &com_pos, Eigen::Vector2d &com_vel, Eigen::Vector2d &com_acc)
{
	// 支持客切り替え時に係数を計算
	if((preview_num%sampling_num_half_cycle == 0 && walk_status < StopWalking) || (preview_num%(sampling_num_half_cycle/2) == 0 && walk_status == StopWalking))
		calcCoefficient();

	float total_time = (walk_status < 2) ? (preview_num % sampling_num_half_cycle) * dt : ((preview_num - last_step_start_time) % (sampling_num_half_cycle * 2)) * dt;

	// 重心位置のアップデート
	com_pos << 0.f, 0.f;
	for(int i=0;i<6;i++){
		com_pos[0] += target_corr[i] * std::pow(total_time, i);
		com_pos[1] += target_corr[i+6] * std::pow(total_time, i);
	}

	// 重心速度のアップデート
	com_vel << 0.f, 0.f;
	for(int i=1;i<6;i++){
		com_vel[0] += i * target_corr[i] * std::pow(total_time, i-1);
		com_vel[1] += i * target_corr[i+6] * std::pow(total_time, i-1);
	}

	// 重心加速度のアップデート
	com_acc << 0.f, 0.f;
	for(int i=2;i<6;i++){
		com_acc[0] += i * (i-1) * target_corr[i] * std::pow(total_time, i-2);
		com_acc[1] += i * (i-1) * target_corr[i+6] * std::pow(total_time, i-2);
	}

	if(walk_status == StopWalkingLastStep && (preview_num - last_step_start_time) == sampling_num_half_cycle/2) foot_y = 0.f;
}

// 重心位置・速度・加速度を取得
bool PreCalculatedPreviewControl::update(Eigen::Vector2d &com_pos, Eigen::Vector2d &com_vel, Eigen::Vector2d &com_acc)
{
	if(end_time < preview_num) return false;
	
	calc_xk(com_pos, com_vel, com_acc);

	preview_num++;

	return true;
}