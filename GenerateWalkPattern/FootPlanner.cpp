#include "FootPlanner.h"

double _rad2deg(double radian){
    return radian * 180/3.14;
}

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
    cout << "stridex,y,th: " << stride.x <<","<<stride.y<<","<<stride.th<<endl;
    if(target_pos.th != 0.0){
        th_foot_step_count = static_cast<int>((fabs(target_pos.th) + stride.th - 0.001) / stride.th);
        cout << "th: normal: " << th_foot_step_count <<"," << foot_step_count << endl;
        if(target_pos.th < 0.0) stride.th *= -1.0f;

        if(th_foot_step_count == foot_step_count){

        }else if(th_foot_step_count < foot_step_count){
            /*
            cout << "####before:" << stride.th << endl;
            stride.th = target_pos.th / foot_step_count;
            cout << "####after:" << stride.th << endl;
            */
            foot_step_count = th_foot_step_count;
        }else{
            foot_step_count = th_foot_step_count;
        }

    }else{
        stride.th = 0.0;
    }
    cout << "result stridex,y,th: " << stride.x <<","<<stride.y<<","<<stride.th<<endl;
	bool result = target_pos_2_foot_step(ref_leg_sup);

	return result;
}

bool FootPlanner::target_pos_2_foot_step(FootStatus ref_leg_sup)
{
    start_walk(ref_leg_sup);
    next_control_point = {0.0f, 0.0f, stride.th};
    for(int i = 1; i < foot_step_count; i++){
        time += step_time;

        if(next_leg_support == RightLeg){
            next_leg_support = LeftLeg;
            foot_y = 1;
        }else if(next_leg_support == LeftLeg){
            next_leg_support = RightLeg;
            foot_y = -1;
        }

        control_point = {
            next_control_point.x + stride.x*cos(next_control_point.th) + (fabs(zmp_correct_y))*cos(next_control_point.th +foot_y*(M_PI/2)),
            next_control_point.y + stride.x*sin(next_control_point.th) + (fabs(zmp_correct_y))*sin(next_control_point.th +foot_y*(M_PI/2)),
            next_control_point.th
        };
//        cout  <<  time << "," << control_point.x << "," << control_point.y << "," << _rad2deg(control_point.th) << endl;
        foot_step_list.push_back(Vector4d(time, control_point.x, control_point.y , control_point.th));

        next_control_point = {
                    next_control_point.x + stride.x*cos(next_control_point.th),//次の原点
                    next_control_point.y + stride.x*sin(next_control_point.th),
                    next_control_point.th + stride.th //次の旋回角度
        };
    }

//    cout << "loopout :" << time << "," << control_point.x << "," << control_point.y << "," << _rad2deg(control_point.th) << endl;
    stop_walk();

//total foot print check

    cout << endl << "total_foot_print" << endl;
    std::size_t size = foot_step_list.size();
    for(std::size_t i=0; i<size; i++){
        cout << foot_step_list[i][0] <<","<<  foot_step_list[i][1] <<","<< foot_step_list[i][2] <<","<< _rad2deg(foot_step_list[i][3]) << endl;
    }

	return true;
}

double FootPlanner::DistanceTargetPos(Pos2D current_pos)
{
    return static_cast<double>(sqrt(pow((target_pos.x-current_pos.x),2)+pow((target_pos.y),2)));
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
    if(target_pos.th != 0.0){
        next_control_point.th = target_pos.th;
    }else if(target_pos.th == 0.0){
        stride.x = target_pos.x - next_control_point.x;
    }
    if(walking_status != Stop){
        time += step_time;
        if(next_leg_support == RightLeg){
            foot_y = 1;
        }else if(next_leg_support == LeftLeg){
            foot_y = -1;
        }

        control_point = {
            next_control_point.x + stride.x*cos(next_control_point.th) + (fabs(zmp_correct_y))*cos(next_control_point.th +foot_y*(M_PI/2)),
            next_control_point.y + stride.x*sin(next_control_point.th) + (fabs(zmp_correct_y))*sin(next_control_point.th +foot_y*(M_PI/2)),
            next_control_point.th
        };

        next_control_point = {
                    next_control_point.x + stride.x*cos(next_control_point.th),//次の原点
                    next_control_point.y + stride.x*sin(next_control_point.th),
                    next_control_point.th + 0
        };

        foot_step_list.push_back(Vector4d(time, control_point.x, control_point.y, control_point.th));
    }

    time += step_time/2.0f;

//    cout << "control_point:   " << time <<"," << control_point.x <<","<< control_point.y  <<","<< _rad2deg(control_point.th) << endl;
//    cout << "next_control_point:   " << time <<"," << next_control_point.x <<","<< next_control_point.y  <<","<< _rad2deg(next_control_point.th) << endl;
	foot_step_list.push_back(Vector4d(time, next_control_point.x, next_control_point.y, next_control_point.th));
	next_leg_support = BothLeg;

	foot_step_list.push_back(Vector4d(time+2.0f, next_control_point.x, next_control_point.y, next_control_point.th));

}
