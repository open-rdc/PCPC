#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>
#include<fstream>
#include <vector>

static const double FOOT_WIDTH			= 0.050;
static const double WALKING_HALF_CYCLE	= 0.340;
static const double SAMPLING_TIME		= 0.010;
static const int NUM_COEF				= 15;
static const double MAX_X_STEP			= 0.060;
static const double MAX_Y_STEP			= 0.060;
static const int MAX_DEG_STEP          = 9;

struct Target_T {
	float time;
	float x;
	float y;
    float deg;
};

std::string to_string(float val, int digits){
	std::ostringstream oss;
	oss << std::fixed << std::setprecision(digits) << val;
	return oss.str();
}

double rad2deg(double radian){
    return radian * 180/M_PI;
}

double deg2rad(int deg){
    return deg * M_PI / 180;
}

float rotate_sign(float a){
    return (a>0.0)-(a<0.0);
}

int main(int argc, char *argv[])
{
	struct Target_T target[10];
	if (argc < 5 || (argc-1)%4 != 0){
		std::cerr << "Generate walking patterns by the Pre-Calculated Preview Control" << std::endl <<
		"Usage: CheckCoefficient <time> <x> <y> <th>[<time> <x> <y> <th>...]" << std::endl;
		exit(-1);
	}
	int num_target = (argc-1)/4;
	for(int i = 0; i < num_target; i ++){
		target[i].time = atof(argv[i * 4 + 1]);
		target[i].x    = atof(argv[i * 4 + 2]);
		target[i].y    = atof(argv[i * 4 + 3]);
        target[i].deg   = atof(argv[i * 4 + 4]);
	}

	std::string com, prev_com;
	std::string com0 = "./GenerateWalkPattern ";
	FILE *fp;
	int support_foot = 0, step = 0, walking = 0;

    float current_x = 0.0f, current_y = 0.0f;
    float cog_x = 0.0f, cog_y = 0.0f;
	float vel_x = 0.0f, vel_y = 0.0f;
    float init_cog_x = 0.0f, init_cog_y = 0.0f;
    float init_vel_x = 0.0f, init_vel_y = 0.0f;

	float ref_zmp_x = 0.0f, ref_zmp_y = 0.0f, ref_zmp_th = 0.0f;
    float prev_ref_zmp_x = 0.0f, prev_ref_zmp_y = 0.0f, prev_ref_zmp_th = 0.0f;
	float ref_zmp_x_gc = 0.0f, ref_zmp_y_gc = 0.0f, ref_zmp_th_gc = 0.0f;

    float trajectory_x = 0.0f, trajectory_y = 0.0f;
    float length = 0.0f, stride_x = 0.0f, stride_y = 0.0f;
    float length_gc = 0.0f, stride_x_gc = 0.0f, stride_y_gc = 0.0f;

    float modi_x = 0.0f, modi_y = 0.0f;
    float modi_vel_x = 0.0f, modi_vel_y = 0.0f;
    float modi_acc_x = 0.0f, modi_acc_y = 0.0f;

	float target_dx = 0.0f, target_dy = 0.0f, target_dth = 0.0f;
	float corr[NUM_COEF] = {0.0f};
	float foot_y = 0.0f;
	int end_time = 2000, last_step_start_time = 0;

    float step_dx = 0.0f, step_dy = 0.0f;
	float disp_x = 0.0f, disp_y = 0.0f;
	int dir = 1;
	int sampling_num_half_cycle = (WALKING_HALF_CYCLE + SAMPLING_TIME / 2) / SAMPLING_TIME;

    std::vector<std::pair<double,double>> xyth_pts_refZMP;
    std::vector<std::pair<double,double>> xyth_pts_COG;
    std::vector<std::pair<double,double>> xyth_pts_trajectory;
    std::vector<std::pair<double,double>> xyth_cog_start_point;
    std::vector<std::pair<double,double>> xyth_pts_vel;
    std::vector<std::pair<double,double>> xyth_pts_acc;
    std::vector<std::pair<double,double>> xyth_pts_vel_points;

    std::vector<std::pair<double,double>> preview_control_zmp;
    std::vector<std::pair<double,double>> preview_control_cog;
    std::vector<std::pair<double,double>> preview_control_vel;
    std::vector<std::pair<double,double>> preview_control_acc;
    std::vector<std::pair<double,double>> preview_control_vel_points;

    std::ifstream ifs;
    std::string prev_buf, prev_str, buf, str;
    std::string data_name = "45deg.csv";//係数の保存先
    std::string prev_data = "45_prev.csv";//45deg時の予見制御の重心位置・速度・加速度のデータ保存先
    int count = 0, prev_count = 0;

#if 1
/*### 45deg旋回の予見制御を用いた重心軌道の導出 */
    int res;
    prev_com = com0;
    prev_com += to_string(target[0].x, 3) + " ";
    prev_com += to_string(target[0].y, 3) + " ";
    prev_com += to_string(target[0].deg, 1) + " ";
    prev_com += "0.0 0.0 0.0 0.0 0 0 ";
    prev_com += prev_data;
    std::cerr <<  prev_com << std::endl<< std::endl;
    const char *command = prev_com.c_str();
    res = system(command);//コマンド実行
    //std::cout << WIFEXITED(res) << "," << WEXITSTATUS(res) << std::endl;

    ifs.open(prev_data,std::ios::in);/* csv file open */
    if(ifs.fail())
    {
        std::cout << "読み込みに失敗" << std::endl;
    }
    while(getline(ifs,prev_str))
    {
        prev_count++;
    }
    ifs.close();
    ifs.open(prev_data,std::ios::in);
    std::vector<double>cog_data(11,0);
    for(int i=0;i<cog_data.size();i++) cog_data[i]=0.f;
    preview_control_vel_points.push_back(std::make_pair(0, 0 ));
    //#load preview_control_cog_data
    for(int i=0;i<prev_count;i++){
        if(i != 0){
            if(ifs && std::getline(ifs, prev_buf)){
                std::istringstream stream(prev_buf);
                for(int j=0; j<11; j++){
                    std::string num;
                    std::getline(stream, num, ' ');
                    cog_data[j] = std::atof(num.c_str());
                }
            }

            if( cog_data[10] != 0 && i%34 == 0){
                //std::cout << cog_data[0] << ","<< cog_data[7] << ","<< cog_data[8] <<std::endl;
                preview_control_vel_points.push_back(std::make_pair(cog_data[7], cog_data[8] ));
            }
        }

        preview_control_zmp.push_back(std::make_pair(cog_data[2], cog_data[5] ));
        preview_control_cog.push_back(std::make_pair(cog_data[3], cog_data[6] ));
        preview_control_vel.push_back(std::make_pair(cog_data[7], cog_data[8] ));
        preview_control_acc.push_back(std::make_pair(cog_data[9], cog_data[10] ));
    }
    ifs.close();
#endif

/*### 多項式の係数を呼び出し */
    ifs.open(data_name,std::ios::in);/* csv file open */
    if(ifs.fail())
    {
        std::cout << "読み込みに失敗" << std::endl;
    }
    while(getline(ifs,str))
    {
        count++;
    }
    ifs.close();
    ifs.open(data_name,std::ios::in);
    std::vector<double>data(15,0);//12->15
    for(int i=0;i<data.size();i++) data[i]=0.f;

	std::cout << "time(s),ref_x(m),ref_y(m),cog_x(m),cog_y(m),v_x(m/s),v_y(m/s),a_x(m/s2),a_y(m/s2)" << std::endl;
    int q = 0;
    int target_deg =0, flag_x = 0;
    float next_trajectory_x = 0.0f, next_trajectory_y = 0.0f, next_trajectory_th = 0.0f;
    int sign = 0;

    xyth_pts_COG.push_back(std::make_pair(0.0, 0.0));
    xyth_pts_vel.push_back(std::make_pair(0.0, 0.0));
    xyth_pts_acc.push_back(std::make_pair(0.0, 0.0));

    /*
		Main Loop
		walking: 	0: 歩行開始
					1: 歩行中
					2: 歩行停止
	*/
	for(int i = 0; i < end_time; i ++){
        // 現在時刻の計算
		float t = SAMPLING_TIME * i;
        // ロボットが歩行中
		// 完全停止するフェーズの一個前
		// 支持客切り替え時
		// である場合は以下を実行
		if ((i%sampling_num_half_cycle  == 0 && walking < 2) || (i%(sampling_num_half_cycle/2) == 0 && walking == 2)){
			if (step < num_target - 1){
				if (t > target[step + 1].time){
                    step ++;
                    flag_x = 0;
                }
			}

            // 以下、このコード内でGenerateWalkPattenを実行するための引数を計算

            // 旋回角を与えられた場合、の残り移動距離計算 //
            //std::cout << "sign  :" << sign << std::endl;
            if(target[step].deg != 0.0 && flag_x == 0){
                target_deg = fabs(target[step].deg);
                while(target_deg>MAX_DEG_STEP){
                    target_deg -= MAX_DEG_STEP;
                    q++;
                }
                if(target_deg != 0) q+= 1;//9degで割れない場合は1ステップ追加する
                //std::cout << q << std::endl;
                target[step].x = ((target[step].x < 0.0 )?(-1):(1))*MAX_X_STEP*q;
                flag_x = 1;
            }
            target[step].x > 0.0 ? sign = 1 : sign = -1; // target[step].xが負のときマイナス符号を係数に掛ける
            if(target[step].deg == 0.0 && flag_x == 0) flag_x = 1;

            //旋回軌道の計算
            trajectory_x  = -(-foot_y)*sin(ref_zmp_th)+ref_zmp_x;//next tarjectory中心の座標系
            trajectory_y  =  (-foot_y)*cos(ref_zmp_th)+ref_zmp_y;
            length  = trajectory_x*cos(-ref_zmp_th) - (trajectory_y*sin(-ref_zmp_th)); //tarjectory移動量
            //std::cout << "steide_x, stride_y   :" << to_string(ref_zmp_x_gc,3)<<"," <<to_string(ref_zmp_y_gc,3) <<", " << to_string(rad2deg(ref_zmp_th_gc),2) << std::endl;

			com = com0;
            // 歩行停止フェーズ出ない場合は以下を実行
            next_trajectory_x = length*cos(ref_zmp_th_gc+ref_zmp_th)+ref_zmp_x_gc;
            next_trajectory_y = length*sin(ref_zmp_th_gc+ref_zmp_th)+ref_zmp_y_gc+(target[step-1].deg != 0 ? 0 : (ref_zmp_y - foot_y));
            next_trajectory_th= ref_zmp_th_gc+ref_zmp_th;

            //std::cout <<"next_trajectory_pos  :"\
            << next_trajectory_x << "," \
            << next_trajectory_y << "," \
            << to_string(rad2deg(next_trajectory_th),2) << std::endl;

			if (walking < 2){
				foot_y = FOOT_WIDTH * ((walking == 0) ? 0 : ((support_foot == 1) ? -1 : 1));

                if(target[step].deg == 0){//旋回角無しの場合

                    target_dx = target[step].x - ref_zmp_x_gc - ref_zmp_x;
                    target_dy = target[step].y - ref_zmp_y_gc - ref_zmp_y - foot_y;

                }else{//旋回角有りの場合
                    //std::cout << "steide_x, stride_y   :" << to_string(stride_x,3)<<"," <<to_string(stride_y,3) << std::endl;
#if 1
                    if(flag_x == 1){//stepが切り替わった最初の場合
                        target_dx = target[step].x;// - length;
                        length_gc = 0;
                        stride_x_gc = target[step-1].x - target_dx;
                        stride_y_gc = target[step-1].y - target_dy;
                        flag_x = 2;
                    }else{//step切り替わり以降
                        target_dx = target[step].x - length_gc - length;
                        length_gc += length;
                        stride_x_gc += stride_x;
                        stride_y_gc += stride_y;
                    }
#endif
                    target_dy = target[step].y;
                }
                if(target[step].deg != 0){
                    target_dth= deg2rad(target[step].deg)- ref_zmp_th_gc - ref_zmp_th;//現在の旋回角
                }else{
                    target_dth = 0.0;
                }
                //std::cout << "time stride_x_gc, stride_y_gc  :" << t <<", " << stride_x_gc << "," << stride_y_gc << std::endl;
/*
                //現在の重心位置の座標移動
                prev_trajectory_x  = - (foot_y)*sin( ref_zmp_th ) + ref_zmp_x;
                prev_trajectory_y  =   (foot_y)*cos( ref_zmp_th ) + ref_zmp_y;
                cog_x = current_x - prev_trajectory_x - ref_zmp_x_gc;
                cog_y = current_y - prev_trajectory_y - ref_zmp_y_gc;

                prev_trajectory をtrajectoryに置き換え、廃止
*/
                //現在の重心位置の座標移動
                trajectory_x  = - (foot_y)*sin( ref_zmp_th ) + ref_zmp_x;
                trajectory_y  =   (foot_y)*cos( ref_zmp_th ) + ref_zmp_y;
                cog_x = current_x - trajectory_x - ref_zmp_x_gc;
                cog_y = current_y - trajectory_y - ref_zmp_y_gc;

                //現在の重心位置の回転座標
                init_cog_x   = (cog_x)*cos(-ref_zmp_th) - (cog_y)*sin(-ref_zmp_th);//next tarjectory中心の座標系
                init_cog_y   = (cog_x)*sin(-ref_zmp_th) + (cog_y)*cos(-ref_zmp_th);
#if 1
                //現在の重心速度
                init_vel_x   = (vel_x)*cos(-ref_zmp_th)-(vel_y)*sin(-ref_zmp_th);
                init_vel_y   = (vel_x)*sin(-ref_zmp_th)+(vel_y)*cos(-ref_zmp_th);
#endif
			} else foot_y = 0;

            // GenerateWalkPatternの引数として追加
			com += to_string(target_dx, 3) + " ";
			com += to_string(target_dy, 3) + " ";
            com += to_string(rad2deg(target_dth), 1) + " ";
			com += to_string(init_vel_x, 4) + " ";
			com += to_string(init_vel_y, 4) + " ";
			com += to_string(init_cog_x, 4) + " ";
			com += to_string(init_cog_y, 4) + " ";
			com += std::to_string(support_foot) + " ";
//            std::cout << "time, ref_zmp_th_gc, ref_zmp_th, prev_ref_zmp_th >= " \
            << t <<"," << ref_zmp_th_gc <<"," << ref_zmp_th << "," << prev_ref_zmp_th << std::endl;
#if 1
            // 歩行フェーズが歩行中かつ旋回角有りかつ目標旋回角が0.01deg未満だった場合は以下を実行
            if (walking == 1 && (fabs(target[step].deg)!=0) && fabs(rad2deg(target_dth)) < 0.01f ){
				walking = 2;
				end_time = i + sampling_num_half_cycle * 2;
				last_step_start_time = i;
                target[step].x = 0.0;

            // 歩行フェーズが歩行中かつ旋回角有りかつ目標位置が0.01m未満だった場合は以下を実行
            }else if (walking == 1 && fabs(target_dx) < 0.01f && fabs(target_dy) < 0.01f){
                walking = 2;
                end_time = i + sampling_num_half_cycle * 2;
                last_step_start_time = i;
                target[step].x = 0.0;
			} else if (walking == 2) walking = 3;
#endif
			com += std::to_string(walking) + " ";
//			com += "temp.csv";
#if 1
			std::cerr <<  com << " test.csv" << "   step: " << step << std::endl<< std::endl;
#endif
            // GenerateWalkPatternの実行
			if ((fp = popen(com.c_str(), "r")) == NULL){
				std::cerr <<  "error" << std::endl;
				exit(-1);
			}
            // GenerateWalkPatternで得られる多項式の係数を取得
			for(int j = 0; j < NUM_COEF; j ++){
				char str[256], *ptr;
				fgets(str, 256, fp);
				if (feof(fp)) break;
				ptr=strchr(str, '\n');
				if (ptr != NULL) *ptr = '\0';
				corr[j] = atof(str);

//				printf("%s\n", str);
			}

            // 以下がPre-Calculated Preview Control の実態部分
			// 上記でFootStepPlannerがGenerateWalkPattern内で実行されているため,PCPC単体で動かすには事前にFootStepPlannerの結果を入れておく必要がある
			if (walking < 3){
                //disp_x = current_x - ref_zmp_x_gc - ref_zmp_x;          //cog_x
				//disp_y = current_y - ref_zmp_y_gc - ref_zmp_y - foot_y; //cog_y
                disp_x = init_cog_x;
                disp_y = init_cog_y;
                dir = (support_foot == 1) ? -1 : 1;

				prev_ref_zmp_x = ref_zmp_x;
				prev_ref_zmp_y = ref_zmp_y;
                prev_ref_zmp_th= ref_zmp_th;

				ref_zmp_x = corr[0];//
				ref_zmp_y = corr[1];
                ref_zmp_th= corr[2];
/*
                prev_trajectory_x  = - (foot_y)*sin( prev_ref_zmp_th ) + prev_ref_zmp_x;
                prev_trajectory_y  =   (foot_y)*cos( prev_ref_zmp_th ) + prev_ref_zmp_y;
                trajectory_stride  = prev_trajectory_x*cos(-prev_ref_zmp_th) - (prev_trajectory_y*sin(-prev_ref_zmp_th));
                std::cout << "tarjectory_stride  " << trajectory_stride << std::endl;

                STEP3
                ref_zmp_th_gc += prev_ref_zmp_th;//累積旋回角 9, 18, 27 ....
                ref_zmp_x_gc  += trajectory_stride*cos(ref_zmp_th_gc);
                ref_zmp_y_gc  += trajectory_stride*sin(ref_zmp_th_gc) + ((prev_ref_zmp_th != 0.0) ? 0.0f : (prev_ref_zmp_y + foot_y));//旋回しながら横移動は考えないものとする
                xyth_pts_trajectory.push_back(std::make_pair(ref_zmp_x_gc, ref_zmp_y_gc ));

                trajectory_stride = length で代用可能だった為廃止
*/
                ref_zmp_th_gc += prev_ref_zmp_th;//累積旋回角 9, 18, 27 ....
                ref_zmp_x_gc  += length*cos(ref_zmp_th_gc);
                ref_zmp_y_gc  += length*sin(ref_zmp_th_gc) + ((prev_ref_zmp_th != 0.0) ? 0.0f : (prev_ref_zmp_y + foot_y));//旋回しながら横移動は考えないものとする
                xyth_pts_trajectory.push_back(std::make_pair(ref_zmp_x_gc, ref_zmp_y_gc ));

#if 1 //旋回に対応させる

				step_dx = (walking == 1) ? ref_zmp_x	 : 0;
				step_dy = (walking == 1) ? ref_zmp_y - foot_y : 0;
#endif
#if 0
                step_dx = (walking == 1) ? ((target[step].deg == 0)? ref_zmp_x : length	 ): 0;
                step_dy = (walking == 1) ? ((target[step].deg == 0)? ref_zmp_y - foot_y : 0): 0;
#endif
			}

            std::cout << "step_dx , step_dy   :" << step_dx << ","<< step_dy << std::endl;
            std::cout << "disp   " << disp_x <<"," << disp_y << std::endl;

            //多項式の係数を45deg.csvから呼び出し( 45deg.csvは周期(0.34)毎)
#if 0
            //#load coefficient
            if(ifs && std::getline(ifs, buf)){
                std::istringstream stream(buf);
                for(int j=0; j<15; j++){
                    std::string num;
                    std::getline(stream, num, ' ');
                    data[j] = std::atof(num.c_str());
                    //std::cout << data[j] << std::endl;
                }
                //std::cout << std::endl;
            }
                //std::cout << "################" << std::endl;
            for(int n=3; n<=14; n++){
                corr[n] = data[n];
                //std::cout << corr[n] << std::endl;
            }
            //std::cout << "################" << std::endl;
#endif
#if 0
                std::cout << "target_dx  = " << target_dx<< std::endl;
			if (walking == 0){
                std::cout << "walking =0" << std::endl << std::endl;
                std::cout << "vel_x, disp_x  :" << vel_x << "," << disp_x << std::endl;
                corr[ 3] = 0.00986713 * vel_x +   1.00095 * disp_x + 0.00000150557 *sign;
				corr[ 4] =   0.960351 * vel_x + 0.0784907 * disp_x + 9.39E-05 *sign;
				corr[ 5] =   -5.88016 * vel_x -   18.2986 * disp_x + 0.0389385 *sign;
				corr[ 6] =    16.6469 * vel_x +   67.6476 * disp_x - 0.0537971 *sign;
				corr[ 7] =   -25.9567 * vel_x -   117.006 * disp_x + 0.109814 *sign;
				corr[ 8] =    18.0493 * vel_x +   85.7187 * disp_x + 0.04934 *sign;
				corr[ 9] = 0.00986713 * vel_y +   1.00095 * disp_y - 0.00000447877;
				corr[10] =   0.960352 * vel_y + 0.0784909 * disp_y - 0.000996702;
				corr[11] =   -5.88016 * vel_y -   18.2986 * disp_y - 0.172952;
				corr[12] =    16.6469 * vel_y +   67.6476 * disp_y + 0.255042;
				corr[13] =   -25.9567 * vel_y -   117.006 * disp_y - 0.541707;
				corr[14] =    18.0493 * vel_y +   85.7184 * disp_y - 0.171364;

			} else if (walking == 1 && fabs(target_dx) >= 0.18){
                std::cout << "walking =1 && target_dx >= 0.18" << std::endl << std::endl;
                std::cout << "vel_x, disp_x  :" << vel_x << "," << disp_x << std::endl;

				corr[ 3] = 0.00986713 * vel_x +   1.00095 * disp_x + 0.00000683605 *sign;
				corr[ 4] =   0.960351 * vel_x + 0.0784907 * disp_x + 0.00210656 *sign;
				corr[ 5] =   -5.88016 * vel_x -   18.2986 * disp_x + 0.308368 *sign;
				corr[ 6] =    16.6469 * vel_x +   67.6476 * disp_x - 0.481904 *sign;
				corr[ 7] =   -25.9567 * vel_x -   117.006 * disp_x + 1.07158 *sign;
				corr[ 8] =    18.0493 * vel_x +   85.7187 * disp_x + 0.165453 *sign;
				corr[ 9] = 0.00986713 * vel_y +   1.00095 * disp_y + 0.000113909 * step_dy + 0.0000566483 * dir;
				corr[10] =   0.960352 * vel_y + 0.0784909 * disp_y +   0.0351079 * step_dy + 0.00659848   * dir;
				corr[11] =   -5.88016 * vel_y -   18.2986 * disp_y +      5.1395 * step_dy - 0.517454     * dir;
				corr[12] =    16.6469 * vel_y +   67.6476 * disp_y -     8.03195 * step_dy + 2.77058      * dir;
				corr[13] =   -25.9567 * vel_y -   117.006 * disp_y +    17.86045 * step_dy - 4.50712      * dir;
				corr[14] =    18.0493 * vel_y +   85.7184 * disp_y +     2.75671 * step_dy + 4.55405      * dir;
			} else if (walking == 1 && fabs(target_dx) >= 0.12){
                std::cout << "walking =1 && target_dx >= 0.12" << std::endl << std::endl;
				corr[ 3] =  0.00986713  * vel_x + 1.000954277 * disp_x + 1.27229E-05 * target_dx + 4.54573E-06 *sign;
				corr[ 4] =  0.960350333 * vel_x + 0.078490216 * disp_x - 0.002620714 * target_dx + 0.002578286 *sign;
				corr[ 5] = -5.880161404 * vel_x - 18.29858372 * disp_x + 0.066635714 * target_dx + 0.2963735   *sign;
				corr[ 6] =  16.64685105 * vel_x + 67.64731602 * disp_x + 0.060257143 * target_dx - 0.492752143 *sign;
				corr[ 7] = -25.95670404 * vel_x - 117.004961  * disp_x - 0.383785714 * target_dx + 1.140670714 *sign;
				corr[ 8] =  18.04929175 * vel_x + 85.71714892 * disp_x + 0.784332143 * target_dx + 0.024265179 *sign;
				corr[ 9] = 0.00986713 * vel_y +   1.00095 * disp_y + 0.000113909 * step_dy + 0.0000566483 * dir;
				corr[10] =   0.960352 * vel_y + 0.0784909 * disp_y +   0.0351079 * step_dy + 0.00659848   * dir;
				corr[11] =   -5.88016 * vel_y -   18.2986 * disp_y +      5.1395 * step_dy - 0.517454     * dir;
				corr[12] =    16.6469 * vel_y +   67.6476 * disp_y -     8.03195 * step_dy + 2.77058      * dir;
				corr[13] =   -25.9567 * vel_y -   117.006 * disp_y +    17.86045 * step_dy - 4.50712      * dir;
				corr[14] =    18.0493 * vel_y +   85.7184 * disp_y +     2.75671 * step_dy + 4.55405      * dir;
			} else if (walking == 1 && fabs(target_dx) >= 0.06){
                std::cout << "walking =1 && target_dx >= 0.06" << std::endl << std::endl;;
				corr[ 3] =  0.00986713  * vel_x + 1.000954277 * disp_x + 1.23591E-05 * target_dx + 4.58932E-06 *sign;
				corr[ 4] =  0.960350333 * vel_x + 0.078490216 * disp_x + 0.004185036 * target_dx + 0.001761604 *sign;
				corr[ 5] = -5.880161404 * vel_x - 18.29858372 * disp_x + 0.582346429 * target_dx + 0.234488107 *sign;
				corr[ 6] =  16.64685105 * vel_x + 67.64731602 * disp_x - 0.956839286 * target_dx - 0.370698321 *sign;
				corr[ 7] = -25.95670404 * vel_x - 117.004961  * disp_x + 2.213864286 * target_dx + 0.828943357 *sign;
				corr[ 8] =  18.04929175 * vel_x + 85.71714892 * disp_x + 0.038196429 * target_dx + 0.113811464 *sign;
				corr[ 9] =  0.00986713  * vel_y + 1.00095     * disp_y + 0.000050603 * target_dy + 0.0000555978 * dir;
				corr[10] =  0.960352    * vel_y + 0.0784912   * disp_y +   0.0188648 * target_dy +   0.00693576 * dir;
				corr[11] = -5.88016     * vel_y - 18.2986     * disp_y +     2.53641 * target_dy -     0.513661 * dir;
				corr[12] =  16.6469     * vel_y + 67.6476     * disp_y -     4.04599 * target_dy +      2.74737 * dir;
				corr[13] = -25.9567     * vel_y - 117.006     * disp_y +     9.12168 * target_dy -      4.42899 * dir;
				corr[14] =  18.0493     * vel_y + 85.7184     * disp_y +    0.986705 * target_dy +      4.47628 * dir;
			} else if (walking == 1){
                std::cout << "walking =1" << std::endl << std::endl;;
				corr[ 3] =  0.00986713  * vel_x + 1.000954277 * disp_x + 8.88479E-05 * target_dx - 2.2516E-22  *sign;
				corr[ 4] =  0.960350333 * vel_x + 0.078490216 * disp_x + 0.033544948 * target_dx - 3.581E-20   *sign;
				corr[ 5] = -5.880161404 * vel_x - 18.29858372 * disp_x + 4.490487582 * target_dx - 4.79369E-18 *sign;
				corr[ 6] =  16.64685105 * vel_x + 67.64731602 * disp_x - 7.135199341 * target_dx + 7.61698E-18 *sign;
				corr[ 7] = -25.95670404 * vel_x - 117.004961  * disp_x + 16.0297956  * target_dx - 3.18075E-20 *sign;
				corr[ 8] =  18.04929175 * vel_x + 85.71714892 * disp_x + 1.934754945 * target_dx - 2.06539E-18 *sign;
				corr[ 9] =  0.00986713  * vel_y + 1.00095     * disp_y + 0.000050603 * target_dy + 0.0000555978 * dir;
				corr[10] =  0.960352    * vel_y + 0.0784912   * disp_y +   0.0188648 * target_dy +   0.00693576 * dir;
				corr[11] = -5.88016     * vel_y - 18.2986     * disp_y +     2.53641 * target_dy -     0.513661 * dir;
				corr[12] =  16.6469     * vel_y + 67.6476     * disp_y -     4.04599 * target_dy +      2.74737 * dir;
				corr[13] = -25.9567     * vel_y - 117.006     * disp_y +     9.12168 * target_dy -      4.42899 * dir;
				corr[14] =  18.0493     * vel_y + 85.7184     * disp_y +    0.986705 * target_dy +      4.47628 * dir;
			} else if ((walking == 2)&&((i - last_step_start_time) < sampling_num_half_cycle / 2)){ // 1st half
                std::cout << "walking =2 1st half" << std::endl << std::endl;;
				corr[ 3] =  0.009842334 * vel_x + 1.000792528 * disp_x + 0.000343766 * step_dx *sign;
				corr[ 4] =  0.964866596 * vel_x + 0.110185519 * disp_x + 0.076905169 * step_dx *sign;
				corr[ 5] = -6.028117018 * vel_x - 19.41137766 * disp_x + 13.02902198 * step_dx *sign;
				corr[ 6] =  18.41986175 * vel_x + 82.09255844 * disp_x - 25.5853022  * step_dx *sign;
				corr[ 7] = -34.89211035 * vel_x - 196.8393784 * disp_x + 69.92306374 * step_dx *sign;
				corr[ 8] =  34.23064825 * vel_x + 246.0010589 * disp_x - 29.83588571 * step_dx *sign;
				corr[ 9] =  0.009842334 * vel_y + 1.000792528 * disp_y + 0.000343766 * step_dy + 5.68033E-05 * dir;
				corr[10] =  0.964866596 * vel_y + 0.110185519 * disp_y + 0.076905169 * step_dy + 0.009354599 * dir;
				corr[11] = -6.028117018 * vel_y - 19.41137766 * disp_y + 13.02902198 * step_dy - 0.319110648 * dir;
				corr[12] =  18.41986175 * vel_y + 82.09255844 * disp_y - 25.5853022  * step_dy + 2.825179693 * dir;
				corr[13] = -34.89211035 * vel_y - 196.8393784 * disp_y + 69.92306374 * step_dy - 6.344316758 * dir;
				corr[14] =  34.23064825 * vel_y + 246.0010589 * disp_y - 29.83588571 * step_dy + 10.80424676 * dir;
			} else { // 2nd half
                std::cout << "walking =2 2nd half" << std::endl << std::endl;;
				corr[ 3] =  0.02698325  * vel_x + 1.079618597 * disp_x - 0.536945198 * step_dx *sign;
				corr[ 4] =  0.634442754 * vel_x - 1.432506511 * disp_x + 7.118750879 * step_dx *sign;
				corr[ 5] = -3.377784333 * vel_x - 6.561093506 * disp_x - 15.05317473 * step_dx *sign;
				corr[ 6] =  6.899324035 * vel_x + 21.28732727 * disp_x + 18.11159451 * step_dx *sign;
				corr[ 7] = -6.651343333 * vel_x - 23.81345498 * disp_x - 12.23722527 * step_dx *sign;
				corr[ 8] =  2.531252298 * vel_x + 9.71487368  * disp_x + 3.636932747 * step_dx *sign;
				corr[ 9] =  0.02698325  * vel_y + 1.079618597 * disp_y - 0.536945198 * step_dy - 0.022866289 * dir;
				corr[10] =  0.634442754 * vel_y - 1.432506511 * disp_y + 7.118750879 * step_dy + 0.284312242 * dir;
				corr[11] = -3.377784333 * vel_y - 6.561093506 * disp_y - 15.05317473 * step_dy - 1.080712981 * dir;
				corr[12] =  6.899324035 * vel_y + 21.28732727 * disp_y + 18.11159451 * step_dy + 1.969947393 * dir;
				corr[13] = -6.651343333 * vel_y - 23.81345498 * disp_y - 12.23722527 * step_dy - 1.802537787 * dir;
				corr[14] =  2.531252298 * vel_y + 9.71487368  * disp_y + 3.636932747 * step_dy + 0.667591974 * dir;
			}
#endif
            for(int i = 3; i<15; i++){
                std::cout << "No." << i << "  = " << corr[i] << std::endl;
            }

            corr[3] += ref_zmp_x_gc;
            corr[9] += ref_zmp_y_gc;


			if (walking < 2){
				vel_x = vel_y = 0.0f;
				for(int j = 1; j < 6; j ++){
					vel_x += j * corr[j+3] * std::pow(WALKING_HALF_CYCLE, j - 1);//WALKING_HALF_CYCLE
					vel_y += j * corr[j+9] * std::pow(WALKING_HALF_CYCLE, j - 1);
				}
                //vel_x = (vel_x)*cos(-ref_zmp_th)-(vel_y)*sin(-ref_zmp_th);
                //vel_y = (vel_x)*sin(-ref_zmp_th)+(vel_y)*cos(-ref_zmp_th);

				if (walking == 1) support_foot ^= 1;
			}
		 	if (walking == 0) walking = 1;

		}// 係数計算終了

        // 歩行開始から現在までの時間を計算
		float dt = (walking < 2) ? (i % sampling_num_half_cycle) * SAMPLING_TIME : ((i - last_step_start_time) % (sampling_num_half_cycle * 2)) * SAMPLING_TIME;

        // 重心位置の計算
        current_x = current_y = 0.0f;
		for(int j = 0; j < 6; j ++){
			current_x += corr[j+3] * std::pow(dt, j);
			current_y += corr[j+9] * std::pow(dt, j);
		}

        //重心位置の軌道接続　旋回
        modi_x = (current_x-ref_zmp_x_gc)*cos(ref_zmp_th_gc) - (current_y-ref_zmp_y_gc)*sin(ref_zmp_th_gc)+ref_zmp_x_gc;//各歩毎でのtrajectory_pointを中心に旋回角分回転
        modi_y = (current_x-ref_zmp_x_gc)*sin(ref_zmp_th_gc) + (current_y-ref_zmp_y_gc)*cos(ref_zmp_th_gc)+ref_zmp_y_gc;//

		// 重心速度の計算
		double v_x = 0.0, v_y = 0.0;
		for(int j = 1; j < 6; j ++){
			v_x += j * corr[j+3] * std::pow(dt, j - 1);
			v_y += j * corr[j+9] * std::pow(dt, j - 1);
		}
        //重心速度の回転座標反映
        modi_vel_x = (v_x)*cos(ref_zmp_th_gc)-(v_y)*sin(ref_zmp_th_gc);
        modi_vel_y = (v_x)*sin(ref_zmp_th_gc)+(v_y)*cos(ref_zmp_th_gc);
        //std::cout << "modi]  " << prev_ref_zmp_th << std::endl;
        // 重心加速度の計算
		double acc_x = 0.0, acc_y = 0.0;
		for(int j = 2; j < 6; j ++){
			acc_x += j * (j - 1) * corr[j+3] * std::pow(dt, j - 2);
			acc_y += j * (j - 1) * corr[j+9] * std::pow(dt, j - 2);
		}
        //重心速度の回転座標反映
        modi_acc_x = (acc_x)*cos(ref_zmp_th_gc)-(acc_y)*sin(ref_zmp_th_gc);
        modi_acc_y = (acc_x)*sin(ref_zmp_th_gc)+(acc_y)*cos(ref_zmp_th_gc);

		if (walking == 2 && (i - last_step_start_time) == 34 / 2) foot_y = 0.0f;

        if(i%34 == 1){
            xyth_cog_start_point.push_back(std::make_pair(modi_x, modi_y ));
            xyth_pts_vel_points.push_back(std::make_pair(modi_vel_x, modi_vel_y ));

        }

        xyth_pts_COG.push_back(std::make_pair(modi_x, modi_y));
        xyth_pts_refZMP.push_back(std::make_pair(ref_zmp_x_gc + fabs(foot_y)*cos(ref_zmp_th_gc+(-1)*((foot_y>0.00)-(foot_y<0.00))*(M_PI/2)), ref_zmp_y_gc + fabs(foot_y)*sin(ref_zmp_th_gc+(-1)*((foot_y>0.00)-(foot_y<0.00))*(M_PI/2))));
        xyth_pts_vel.push_back(std::make_pair(modi_vel_x, modi_vel_y));
        xyth_pts_acc.push_back(std::make_pair(modi_acc_x, modi_acc_y));
	}

// 確認用　描画
#if 1
    FILE *gp = popen("gnuplot -persist\n", "w");
    fprintf(gp, "set term qt 1 \n");
    fprintf(gp, "set title  \"Check Coefficient\"\n");
    fprintf(gp, "set xlabel \"x [m]\"\n");
    //fprintf(gp, "set xrange[-0.05:0.3]\n");
    fprintf(gp, "set ylabel \"y [m]\"\n");
    //fprintf(gp, "set yrange[-0.05:0.2]\n");
    fprintf(gp, "set size ratio -1\n");
    fprintf(gp, "set grid \n");
    fprintf(gp, "set key left top \n");
    fprintf(gp, "plot '-' with lines lw 3 lt 2 title \"RefZMP\" \
                     ,'-' with lines lw 3 lt 7 title \"COM\" \
                     ,'-' with points lw 2 lt 1 pt 7 title \"Foot planner trajectory\" \
                     ,'-' with lines lw 2 lt 6 title \"TJY\" \
                     ,'-' with points lw 3 lt 8 pt 7 title \"COG  START Point\" \
                     ,'-' with lines lw 3 lt 9 dt 3 title \"PreviewControl COG\", \n");

    for(std::size_t i=0;i<xyth_pts_refZMP.size();i++) fprintf(gp, "%f\t%f\n", xyth_pts_refZMP[i].first, xyth_pts_refZMP[i].second); fprintf(gp, "e\n");
    for(std::size_t i=0;i<xyth_pts_COG.size();i++) fprintf(gp, "%f\t%f\n", xyth_pts_COG[i].first, xyth_pts_COG[i].second); fprintf(gp,"e\n");

    for(std::size_t i=0;i<xyth_pts_trajectory.size();i++) fprintf(gp, "%f\t%f\n", xyth_pts_trajectory[i].first, xyth_pts_trajectory[i].second); fprintf(gp,"e\n");
    for(std::size_t i=0;i<xyth_pts_trajectory.size();i++) fprintf(gp, "%f\t%f\n", xyth_pts_trajectory[i].first, xyth_pts_trajectory[i].second); fprintf(gp,"e\n");
    for(std::size_t i=0;i<xyth_cog_start_point.size();i++) fprintf(gp, "%f\t%f\n", xyth_cog_start_point[i].first, xyth_cog_start_point[i].second); fprintf(gp,"e\n");
    for(std::size_t i=0;i<preview_control_cog.size();i++) fprintf(gp, "%f\t%f\n", preview_control_cog[i].first, preview_control_cog[i].second); fprintf(gp,"e\n");

    fprintf(gp,"exit\n");
    pclose(gp);
#endif

#if 1
#if 1
    FILE *gpy = popen("gnuplot -persist\n", "w");
    fprintf(gpy, "set term qt 4 \n");
    fprintf(gpy, "set title  \"cog y\"\n");
    fprintf(gpy, "set xlabel \"t [s]\"\n");
    fprintf(gpy, "set xrange[0:3]\n");
    fprintf(gpy, "set ylabel \"y [m]\"\n");
    fprintf(gpy, "set yrange[-0.06:0.15]\n");
    //fprintf(gpy, "set size ratio -1\n");
    fprintf(gpy, "set grid \n");
    fprintf(gpy, "set key left top \n");
    fprintf(gpy, "plot '-' with lines lw 2 lt 2  title \"preview control vel\" \
                        ,'-' with lines lw 2 lt 7 dt 3 title \"pcpc vel\", \n");
    for(std::size_t i=0;i<preview_control_cog.size();i++) fprintf(gpy, "%f\t%f\n", (i*0.01), preview_control_cog[i].second ); fprintf(gpy, "e\n");
    for(std::size_t i=0;i<xyth_pts_COG.size();i++) fprintf(gpy, "%f\t%f\n", (i*0.01), xyth_pts_COG[i].second ); fprintf(gpy, "e\n");
    fprintf(gpy,"exit\n");
    pclose(gpy);
#endif

#if 1
    FILE *gp4 = popen("gnuplot -persist\n", "w");
    fprintf(gp4, "set term qt 4 \n");
    fprintf(gp4, "set title  \"vel xy\"\n");
    fprintf(gp4, "set xlabel \"x [s]\"\n");
    fprintf(gp4, "set ylabel \"y [m]\"\n");
    fprintf(gp4, "set size ratio -1\n");
    fprintf(gp4, "set grid \n");
    fprintf(gp4, "set key left top \n");
    fprintf(gp4, "plot '-' with lines lw 2 lt 2  title \"preview control vel\" \
                        ,'-' with lines lw 2 lt 7 dt 3 title \"pcpc vel\", \n");
    for(std::size_t i=0;i<preview_control_vel.size();i++) fprintf(gp4, "%f\t%f\n",preview_control_vel[i].first, preview_control_vel[i].second ); fprintf(gp4, "e\n");
    for(std::size_t i=0;i<xyth_pts_vel.size();i++) fprintf(gp4, "%f\t%f\n", xyth_pts_vel[i].first, xyth_pts_vel[i].second ); fprintf(gp4, "e\n");
    fprintf(gp4,"exit\n");
    pclose(gp4);
#endif

#if 1
    FILE *gp0 = popen("gnuplot -persist\n", "w");
    fprintf(gp0, "set term qt 2 \n");
    fprintf(gp0, "set title  \"vel x\"\n");
    fprintf(gp0, "set xlabel \"t [s]\"\n");
    fprintf(gp0, "set ylabel \"dx [m]\"\n");
    fprintf(gp0, "set size ratio -1\n");
    fprintf(gp0, "set grid \n");
    fprintf(gp0, "set key right top \n");
    fprintf(gp0, "plot '-' with lines lw 2 lt 2  title \"preview control vel x\"\
                    , '-' with lines lw 2 lt 7 dt 3 title \"pcpc vel x\"\
                    , '-' with points lw 3 lt 8 pt 7 title \"prev vel start point\" \
                    , '-' with points lw 3 lt 9 pt 7 title \"pcpc vel start point\" ,\n");
    for(std::size_t i=0;i<preview_control_vel.size();i++) fprintf(gp0, "%f\t%f\n", (i*0.01), preview_control_vel[i].first ); fprintf(gp0, "e\n");
    for(std::size_t i=0;i<xyth_pts_vel.size();i++) fprintf(gp0, "%f\t%f\n", (i*0.01), xyth_pts_vel[i].first ); fprintf(gp0, "e\n");
    for(std::size_t i=0;i<preview_control_vel_points.size();i++) fprintf(gp0, "%f\t%f\n", (i*0.34), preview_control_vel_points[i].first ); fprintf(gp0, "e\n");
    for(std::size_t i=0;i<xyth_pts_vel_points.size();i++) fprintf(gp0, "%f\t%f\n", (i*0.34), xyth_pts_vel_points[i].first ); fprintf(gp0, "e\n");
    fprintf(gp0,"exit\n");
    pclose(gp0);
#endif
#if 1
    FILE *gp1 = popen("gnuplot -persist\n", "w");
    fprintf(gp1, "set term qt 3 \n");
    fprintf(gp1, "set title  \"vel y\"\n");
    fprintf(gp1, "set xlabel \"t [s]\"\n");
    fprintf(gp1, "set ylabel \"dy [m]\"\n");
    fprintf(gp1, "set size ratio -1\n");
    fprintf(gp1, "set grid \n");
    fprintf(gp1, "set key right top \n");
    fprintf(gp1, "plot '-' with lines lw 2 lt 2  title \"preview control vel y\" \
                      ,'-' with lines lw 2 lt 7 dt 3 title \"pcpc vel y\" \
                      ,'-' with points lw 3 lt 8 pt 7 title \"prev vel start point\" \
                      ,'-' with points lw 3 lt 9 pt 7 title \"pcpc vel start point\" ,\n");
    for(std::size_t i=0;i<preview_control_vel.size();i++) fprintf(gp1, "%f\t%f\n", (i*0.01),preview_control_vel[i].second); fprintf(gp1, "e\n");
    for(std::size_t i=0;i<xyth_pts_vel.size();i++) fprintf(gp1, "%f\t%f\n", (i*0.01), xyth_pts_vel[i].second); fprintf(gp1, "e\n");
    for(std::size_t i=0;i<preview_control_vel_points.size();i++) fprintf(gp0, "%f\t%f\n", (i*0.34), preview_control_vel_points[i].second ); fprintf(gp1, "e\n");
    for(std::size_t i=0;i<xyth_pts_vel_points.size();i++) fprintf(gp0, "%f\t%f\n", (i*0.34), xyth_pts_vel_points[i].second ); fprintf(gp1, "e\n");

    fprintf(gp1,"exit\n");
    pclose(gp1);

#endif
#if 1
    FILE *gp2 = popen("gnuplot -persist\n", "w");
    fprintf(gp2, "set term qt 4 \n");
    fprintf(gp2, "set title  \"acc x\"\n");
    fprintf(gp2, "set xlabel \"t [s]\"\n");
    fprintf(gp2, "set ylabel \"ddx [m]\"\n");
    fprintf(gp2, "set size ratio -1\n");
    fprintf(gp2, "set grid \n");
    fprintf(gp2, "set key left top \n");
    fprintf(gp2, "plot '-' with lines lw 2 lt 2  title \"preview control acc x\" \
                        ,'-' with lines lw 2 lt 7 dt 3 title \"pcpc acc x\", \n");
    for(std::size_t i=0;i<preview_control_acc.size();i++) fprintf(gp2, "%f\t%f\n", (i*0.01), preview_control_acc[i].first ); fprintf(gp2, "e\n");
    for(std::size_t i=0;i<xyth_pts_acc.size();i++) fprintf(gp2, "%f\t%f\n", (i*0.01), xyth_pts_acc[i].first ); fprintf(gp2, "e\n");
    fprintf(gp2,"exit\n");
    pclose(gp2);

#endif
#if 1
    FILE *gp3 = popen("gnuplot -persist\n", "w");
    fprintf(gp3, "set term qt 5 \n");
    fprintf(gp3, "set title  \"acc y\"\n");
    fprintf(gp3, "set xlabel \"t [s]\"\n");
    //fprintf(gp, "set xrange[0:3.5]\n");
    fprintf(gp3, "set ylabel \"ddy [m]\"\n");
    //fprintf(gp, "set yrange[-2.5:2.5]\n");
    fprintf(gp3, "set size ratio -1\n");
    fprintf(gp3, "set grid \n");
    fprintf(gp3, "set key left top \n");
    fprintf(gp3, "plot '-' with lines lw 2 lt 2 title \"preview control acc y\" \
                      ,'-' with lines lw 2 lt 7 dt 3 title \"pcpc acc y\",\n");
    for(std::size_t i=0;i<preview_control_acc.size();i++) fprintf(gp3, "%f\t%f\n", (i*0.01), preview_control_acc[i].second ); fprintf(gp3, "e\n");
    for(std::size_t i=0;i<xyth_pts_acc.size();i++) fprintf(gp3, "%f\t%f\n", (i*0.01), xyth_pts_acc[i].second ); fprintf(gp3, "e\n");
    fprintf(gp3,"exit\n");
    pclose(gp3);
#endif
#endif

	return 0;
}
