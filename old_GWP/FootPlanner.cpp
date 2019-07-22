#include "FootPlanner.h"

bool FootPlanner::SetTargetPos(const double target_x, const double target_y, const double target_th, FootStatus ref_leg_sup, WalkingStatus ref_walking_status)
{
	target_pos.x	 = target_x;
	target_pos.y	 = target_y;
	target_pos.th	 = target_th;
	next_leg_support = ref_leg_sup;
	walking_status	 = ref_walking_status;

	foot_step_list_clear();

	target_distance = sqrt(pow(target_x, 2) + pow(target_y, 2));

	if(target_pos.x == 0 && target_pos.y == 0){
		stride.x = stride.y = 0;
	}else if(fabs(target_pos.x) > fabs(target_pos.y)){
		foot_step_count = static_cast<int>((fabs(target_pos.x) + stride.x - 0.001) / stride.x);
		if(target_pos.x < 0) stride.x *= -1.0f;
		stride.y = target_pos.y / foot_step_count;
	}else{
		foot_step_count = static_cast<int>((fabs(target_pos.y) + stride.y - 0.001) / stride.y);
		if(target_pos.y < 0) stride.y *= -1.0f;
		stride.x = target_pos.x / foot_step_count;
	}

//	cout << "foot_step_stride = "<< stride.x << "," << stride.y << ", " << stride.th << endl;

	bool result = target_pos_2_foot_step(ref_leg_sup);
	return result;
}

bool FootPlanner::target_pos_2_foot_step(FootStatus ref_leg_sup)
{
	int counter = 0;

	start_walk(ref_leg_sup);

	control_point = {0.0f, 0.0f, 0.0f};
	while(1){
		counter++;

		if(DistanceTargetPos(control_point) <= sqrt(pow(stride.x,2)+pow(stride.y,2))) break;
		if(foot_step_count < counter) return false;

		next_control_point = {
			control_point.x  + stride.x,
			control_point.y  + stride.y,
			control_point.th + stride.th
		};

		time += step_time;

		if(next_leg_support == RightLeg){
			foot_step_list.push_back(Vector4d(time, next_control_point.x, next_control_point.y+zmp_correct_y, next_control_point.th));
			next_leg_support = LeftLeg;
		}else if(next_leg_support == LeftLeg){
			foot_step_list.push_back(Vector4d(time, next_control_point.x, next_control_point.y-zmp_correct_y, next_control_point.th));
			next_leg_support = RightLeg;
		}

		control_point = next_control_point;
        //cout << "check" << endl;
	}
//    cout << "loop_out = " << time << "," << control_point.x << "," << control_point.y << "," << control_point.th << endl;
	stop_walk();

	return true;
}

double FootPlanner::DistanceTargetPos(Pos2D current_pos)
{
	return static_cast<double>(sqrt(pow((target_pos.x-current_pos.x),2)+pow((target_pos.y-current_pos.y),2)));
}

void FootPlanner::start_walk(FootStatus ref_leg_sup)
{
	if(walking_status == Start){
		foot_step_list.push_back(Vector4d(0.0f, 0.0f, 0.0f, 0.0f));
		time += step_time;
	}

	if(ref_leg_sup == RightLeg){
		foot_step_list.push_back(Vector4d(time, 0.0f, zmp_correct_y, 0.0f));
		next_leg_support = LeftLeg;
	}else if(ref_leg_sup == LeftLeg){
		foot_step_list.push_back(Vector4d(time, 0.0f, -zmp_correct_y, 0.0f));
		next_leg_support = RightLeg;
	}
}

void FootPlanner::stop_walk()
{
	next_control_point = target_pos;

	if(walking_status != Stop){
		time += step_time;
		if(next_leg_support == RightLeg)
			foot_step_list.push_back(Vector4d(time, next_control_point.x, next_control_point.y+zmp_correct_y, next_control_point.th));
		else if(next_leg_support == LeftLeg)
			foot_step_list.push_back(Vector4d(time, next_control_point.x, next_control_point.y-zmp_correct_y, next_control_point.th));
	}
	time += step_time/2.0f;
	foot_step_list.push_back(Vector4d(time, next_control_point.x, next_control_point.y, next_control_point.th));
	next_leg_support = BothLeg;

	foot_step_list.push_back(Vector4d(time+2.0f, next_control_point.x, next_control_point.y, next_control_point.th));
}
