#include "FootPlanner.h"

bool FootPlanner::SetTargetPos(const double target_x, const double target_y, const double target_th, FootStatus ref_leg_sup, WalkingStatus ref_walking_status)
{
	target_pos.x	 = target_x;
	target_pos.y	 = target_y;
	target_pos.th	 = target_th;
	next_leg_support = ref_leg_sup;
	walking_status	 = ref_walking_status;
    int th_foot_step_count = 0;

	foot_step_list_clear();

	target_distance = sqrt(pow(target_x, 2) + pow(target_y, 2));

    if(target_pos.x == 0 && target_pos.y == 0 && target_pos.th == 0.0){
        stride.x = stride.y = stride.th = 0.0;

    }else if(fabs(target_pos.x) > fabs(target_pos.y)){
        foot_step_count = static_cast<int>((fabs(target_pos.x) + stride.x - 0.001) / stride.x);
        if(target_pos.x < 0) stride.x *= -1.0f;
        stride.y = target_pos.y / foot_step_count;
    }else{
        foot_step_count = static_cast<int>((fabs(target_pos.y) + stride.y - 0.001) / stride.y);
        if(target_pos.y < 0) stride.y *= -1.0f;
        stride.x = target_pos.x / foot_step_count;
    }

    if(target_pos.th != 0.0){
        th_foot_step_count = static_cast<int>((fabs(target_pos.th) + stride.th - 0.001) / stride.th);
        if(target_pos.th < 0.0) stride.th *= -1.0f;

        if(th_foot_step_count == foot_step_count){

        }else if(th_foot_step_count < foot_step_count){
            stride.th = target_pos.th / foot_step_count;

        }else{
            foot_step_count = th_foot_step_count;
        }

    }else{
        stride.th = 0.0;
    }
/*
    cout << endl << "th_foot_count = " << th_foot_step_count << endl;
    cout << endl;
    cout << "total_stride(x,y,th) = "<< stride.x << ","<< stride.y << "," << stride.th * 180/3.14 << "(deg) "<< endl;
    cout << "total_foot_step_count=" << foot_step_count << endl;
*/
	bool result = target_pos_2_foot_step(ref_leg_sup);
	return result;
}

bool FootPlanner::target_pos_2_foot_step(FootStatus ref_leg_sup)
{
    start_walk(ref_leg_sup);

    for(int i = 1; i < foot_step_count; i++){

        //if(DistanceTargetPos(control_point) <= sqrt(pow(stride.x,2)+pow(stride.y,2))) break; //注意
        shift_y = foot_y * ((i%2)*2-1)*2*(-1);
        next_control_point.x = control_point.x + stride.x*cos(control_point.th) - (stride.y + shift_y)*sin(control_point.th);
        next_control_point.y = control_point.y + stride.x*sin(control_point.th) + (stride.y + shift_y)*cos(control_point.th);
        next_control_point.th= control_point.th + stride.th;

        time += step_time;
        foot_step_list.push_back(Vector4d(time, next_control_point.x, next_control_point.y, next_control_point.th));
        control_point = next_control_point;
    }

    stop_walk();

	return true;
}

double FootPlanner::DistanceTargetPos(Pos2D current_pos)
{
    double check = current_pos.y - (current_pos.y);
    return static_cast<double>(sqrt(pow((target_pos.x-current_pos.x),2)+pow((target_pos.y),2)));
}

void FootPlanner::start_walk(FootStatus ref_leg_sup)
{
    if(ref_leg_sup == -1){ //shiftの調整 左右スタート支持脚で注意
        foot_y = zmp_correct_y;
    }else if(ref_leg_sup == 1){
        foot_y = -zmp_correct_y;
    }

    control_point = {0.0f, 0.0f, 0.0f};
    next_control_point = {0.0f, 0.0f, 0.0f};

    if(walking_status == Start){
        foot_step_list.push_back(Vector4d(time, control_point.x, control_point.x, control_point.th));
        time += step_time;
    }

    next_control_point.x  = control_point.x ;
    next_control_point.y  = control_point.y + foot_y;
    next_control_point.th = control_point.th;
    foot_step_list.push_back(Vector4d(time, next_control_point.x, next_control_point.y, control_point.th));

    control_point = next_control_point;
}

void FootPlanner::stop_walk()
{
    next_control_point = target_pos;

    if(shift_y == -0.1){
        next_control_point.x = control_point.x + stride.x*cos(next_control_point.th) - (stride.y - shift_y)*sin(next_control_point.th);
        next_control_point.y = control_point.y + stride.x*sin(next_control_point.th) + (stride.y - shift_y)*cos(next_control_point.th);

        if(walking_status != Stop){
            time += step_time;
            foot_step_list.push_back(Vector4d(time, next_control_point.x, next_control_point.y, next_control_point.th));
        }

    }else if(shift_y == 0.1){
        next_control_point.x = control_point.x + stride.x*cos(next_control_point.th) - (stride.y - shift_y)*sin(next_control_point.th);
        next_control_point.y = control_point.y + stride.x*sin(next_control_point.th) + (stride.y - shift_y)*cos(next_control_point.th);

        if(walking_status != Stop){
            time += step_time;
            foot_step_list.push_back(Vector4d(time, next_control_point.x, next_control_point.y, next_control_point.th));
        }
    }

    time += step_time;

    if(shift_y == -0.1){
        next_control_point.x += zmp_correct_y*cos(next_control_point.th - (M_PI/2));
        next_control_point.y += zmp_correct_y*sin(next_control_point.th - (M_PI/2));
    }else if(shift_y == 0.1){
        next_control_point.x += zmp_correct_y*cos(next_control_point.th + (M_PI/2));
        next_control_point.y += zmp_correct_y*sin(next_control_point.th + (M_PI/2));
    }

    foot_step_list.push_back(Vector4d(time, next_control_point.x, next_control_point.y, next_control_point.th));

    next_leg_support = BothLeg;

    foot_step_list.push_back(Vector4d(time+2.0f, next_control_point.x, next_control_point.y, next_control_point.th));
}
