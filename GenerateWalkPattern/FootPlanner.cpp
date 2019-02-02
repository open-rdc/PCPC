#include "FootPlanner.h"

bool FootPlanner::SetTargetPos(const double target_x, const double target_y, const double target_th, FootStatus ref_leg_sup, WalkingStatus ref_walking_status)
{
	target_pos.x	 = target_x;
	target_pos.y	 = target_y;
	target_pos.th	 = target_th;
	next_leg_support = ref_leg_sup;
	walking_status	 = ref_walking_status;
    start_leg_sup = ref_leg_sup;
    int th_foot_step_count = 0;

	foot_step_list_clear();

	target_distance = sqrt(pow(target_x, 2) + pow(target_y, 2));

    if(target_pos.x == 0 && target_pos.y == 0 && target_pos.th == 0.0){
        stride.x = stride.y = stride.th = 0.0;

    }else if(fabs(target_pos.x) > fabs(target_pos.y)){
        foot_step_count = static_cast<int>((fabs(target_pos.x) + stride.x - 0.001) / stride.x);
//        cout << "create = " << ((fabs(target_pos.x) + stride.x - 0.001) / stride.x) << endl;
        if(target_pos.x < 0) stride.x *= -1.0f;
        stride.y = target_pos.y / foot_step_count;
    }else{
        foot_step_count = static_cast<int>((fabs(target_pos.y) + stride.y - 0.001) / stride.y);
//        cout << "create = " << ((fabs(target_pos.y) + stride.y - 0.001) / stride.y) << endl;
        if(target_pos.y < 0) stride.y *= -1.0f;
        stride.x = target_pos.x / foot_step_count;
    }

//    cout << "foot_step_count=" << foot_step_count << endl;
//    cout << "xy _ stride(x,y,th) = "<< stride.x << ","<< stride.y << endl;

    if(target_pos.th != 0.0){
        th_foot_step_count = static_cast<int>((fabs(target_pos.th) + stride.th - 0.001) / stride.th);
//        cout << "create = " << ((fabs(target_pos.th) + stride.th - 0.001) / stride.th) << endl << endl;
        if(target_pos.th < 0.0) stride.th *= -1.0f;

        if(th_foot_step_count == foot_step_count){
            //cout << "th_foot_step_count = foot_step_count" << endl;

        }else if(th_foot_step_count < foot_step_count){
            stride.th = target_pos.th / foot_step_count;

        }else{
            foot_step_count = th_foot_step_count;
        //    stride.x = target_pos.x / foot_step_count;
        //    stride.y = target_pos.x / foot_step_count;
        }

    }else{
        stride.th = 0.0;
    }
/*
    double stride_distance = sqrt(pow(stride.x, 2) + pow(stride.y, 2));
    next_control_point = {0.0f, 0.0f, 0.0f};
    for(int i = 0; i<foot_step_count; i++){
        next_control_point.x += stride.x * cos(next_control_point.th);// - stride.y * sin(next_control_point.th);
        next_control_point.y += stride.x * sin(next_control_point.th);// + stride.th* cos(next_control_point.th);
        next_control_point.th += stride.th;
        cout << "next_control_point = " << next_control_point.x << "," << next_control_point.y << "," << next_control_point.th << endl;
    }
    target_pos = next_control_point;
*/
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
	int counter = 0;
    double shift_y_right= 0, shift_y_left = 0;

    //cout << "ref_leg_sup = " << ref_leg_sup << endl; //left  : 1 right : -1

    if(ref_leg_sup == -1){ //shiftの調整 左右スタート支持脚で注意
        shift_y_right= 0;
        shift_y_left = -zmp_correct_y;
    }else if(ref_leg_sup == 1){
        shift_y_right= zmp_correct_y;
        shift_y_left = 0;
    }

//	start_walk(ref_leg_sup);


    double shift_y, foot_y;

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

    next_control_point.x  = control_point.x + 0;
    next_control_point.y  = control_point.y + foot_y;
    next_control_point.th = control_point.th;
    foot_step_list.push_back(Vector4d(time, next_control_point.x, next_control_point.y, control_point.th));

    control_point = next_control_point;
    cout << "foot_step_count = " << foot_step_count << endl;


    for(int i = 1; i < foot_step_count; i++){

        //if(DistanceTargetPos(control_point) <= sqrt(pow(stride.x,2)+pow(stride.y,2))) break; //注意
        shift_y = foot_y * ((i%2)*2-1)*2*(-1);
        next_control_point.x = control_point.x + stride.x*cos(control_point.th) - (stride.y + shift_y)*sin(control_point.th);
        next_control_point.y = control_point.y + stride.x*sin(control_point.th) + (stride.y + shift_y)*cos(control_point.th);
        next_control_point.th= control_point.th + stride.th;

        time += step_time;
        foot_step_list.push_back(Vector4d(time, next_control_point.x, next_control_point.y, next_control_point.th));
        control_point = next_control_point;
        cout << "loop_out = " << i << "," << time << "," << control_point.x << "," << control_point.y << "," << control_point.th << endl;
    }
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

    time += step_time;///2.0f;

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



# if 0
    while(1){
		counter++;

        if(target_pos.th != 0.0){
            if((fabs(target_pos.th) - fabs(control_point.th)) <= stride.th){ cout << "fabs(th)"<< endl; break;}
        }else if(target_pos.th == 0.0){
            if(DistanceTargetPos(control_point) <= sqrt(pow(stride.x,2)+pow(stride.y,2)))
            {
                cout << "dist flag  !"<< endl;
                break;
            }
        }
		if(foot_step_count < counter) return false;

		time += step_time;

		if(next_leg_support == RightLeg){

            next_control_point.x  = control_point.x  + stride.x*cos(control_point.th) - (stride.y + zmp_correct_y)*sin(control_point.th);
            next_control_point.y  = control_point.y  + stride.x*sin(control_point.th) + (stride.y + zmp_correct_y)*cos(control_point.th);
            next_control_point.th = control_point.th + stride.th;

			foot_step_list.push_back(Vector4d(time, next_control_point.x, next_control_point.y + shift_y_right, next_control_point.th));
			next_leg_support = LeftLeg;

		}else if(next_leg_support == LeftLeg){

            next_control_point.x  = control_point.x  + stride.x*cos(control_point.th) - (stride.y - zmp_correct_y)*sin(control_point.th);
            next_control_point.y  = control_point.y  + stride.x*sin(control_point.th) + (stride.y - zmp_correct_y)*cos(control_point.th);
            next_control_point.th = control_point.th + stride.th;

			foot_step_list.push_back(Vector4d(time, next_control_point.x, next_control_point.y + shift_y_left, next_control_point.th));//next_control_point.th
			next_leg_support = RightLeg;
		}

		control_point = next_control_point;
	}
    //cout << "loopout = " << time << ", " << control_point.x << "," << control_point.y << "," << control_point.th* 180/3.14 << "(deg) " << endl;

    stop_walk();

#endif
	return true;
}

double FootPlanner::DistanceTargetPos(Pos2D current_pos)
{
    double check = current_pos.y - (current_pos.y);
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
    double shift_y_right= 0, shift_y_left = 0;

    if(start_leg_sup == -1){
        shift_y_right= 0;
        shift_y_left = -zmp_correct_y;
    }else if(start_leg_sup == 1){
        shift_y_right= zmp_correct_y;
        shift_y_left = 0;
    }

    if( target_pos.th != 0.0){
            next_control_point.th = target_pos.th;

    	if(walking_status != Stop){
    		time += step_time;
    		if(next_leg_support == RightLeg){

                next_control_point.x  = control_point.x  + stride.x*cos(next_control_point.th) - (stride.y + zmp_correct_y)*sin(next_control_point.th);
                next_control_point.y  = control_point.y  + stride.x*sin(next_control_point.th) + (stride.y + zmp_correct_y)*cos(next_control_point.th);
    			foot_step_list.push_back(Vector4d(time, next_control_point.x, next_control_point.y + shift_y_right, next_control_point.th));

    		}else if(next_leg_support == LeftLeg){

                next_control_point.x  = control_point.x  + stride.x*cos(next_control_point.th) - (stride.y - zmp_correct_y)*sin(next_control_point.th);
                next_control_point.y  = control_point.y  + stride.x*sin(next_control_point.th) + (stride.y - zmp_correct_y)*cos(next_control_point.th);
    			foot_step_list.push_back(Vector4d(time, next_control_point.x, next_control_point.y + shift_y_left,  next_control_point.th));
            }
    	}

        control_point = next_control_point;
    	time += step_time/2.0f;

            if(next_leg_support == RightLeg){

                next_control_point.x  = control_point.x + stride.x*sin(next_control_point.th);;//next_control_point.x  - ((zmp_correct_y + zmp_correct_y)/2.0f)*sin(next_control_point.th);
                next_control_point.y  = control_point.y + stride.y*cos(next_control_point.th);
                //cout << "Right :" << time << endl;
                foot_step_list.push_back(Vector4d(time, next_control_point.x, (next_control_point.y + shift_y_right), next_control_point.th));

            }else if(next_leg_support == LeftLeg){

                next_control_point.x  = control_point.x  - ((zmp_correct_y + zmp_correct_y)/2.0f)*sin(next_control_point.th);
                next_control_point.y  = control_point.y  + ((zmp_correct_y + zmp_correct_y)/2.0f)*cos(next_control_point.th);
                //cout << "Left :" << time << endl;
                foot_step_list.push_back(Vector4d(time, next_control_point.x, (next_control_point.y + shift_y_left),  next_control_point.th));
            }

    	next_leg_support = BothLeg;

    	foot_step_list.push_back(Vector4d(time+2.0f, next_control_point.x, next_control_point.y, next_control_point.th));

    }else{

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

}

#if 0
void FootPlanner::stop_walk()
{
    control_point = {0.0, 0.0, 0.0};

        control_point.x = (target_pos.x - step.x * foot_step_count)*cos(next_control_point.th)-(target_pos.y - step.y * foot_step_count - shift_y)*sin(next_control_point.th);
        control_point.y = (target_pos.x - step.x * foot_step_count)*sin(next_control_point.th)+(target_pos.y - step.y * foot_step_count - shift_y)*cos(next_control_point.th);

        next_control_point.x  += control_point.x;
        next_control_point.y  += control_point.y;
        next_control_point.th += control_point.th;

        control_point.x = -(shift_y/2)*sin(next_control_point.th);
        control_point.y = (shift_y/2)*cos(next_control_point.th);
        next_control_point.x  += control_point.x;
        next_control_point.y  += control_point.y;

    if(walking_status != Stop){
        time += step_time/2.0f;
        foot_step_list.push_back(Vector4d(time, next_control_point.x, next_control_point.y, next_control_point.th));//next_control_point.th));

    }else{

        time = time - step_time/2.0;
        foot_step_list[foot_step_count] = {time, next_control_point.x, next_control_point.y, next_control_point.th};

    }

    next_leg_support = BothLeg;

	foot_step_list.push_back(Vector4d(time+2.0f, next_control_point.x, next_control_point.y, next_control_point.th));

}
#endif
