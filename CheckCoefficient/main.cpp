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

struct Target_T {
	float time;
	float x;
	float y;
    float th;
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
        target[i].th   = deg2rad(atof(argv[i * 4 + 4]));
	}

	std::string com, prev_com;
	std::string com0 = "./GenerateWalkPattern ";
	FILE *fp;
	int support_foot = 0, step = 0, walking = 0;
	float current_x = 0.0f, current_y = 0.0f;
	float vel_x = 0.0f, vel_y = 0.0f;
	float ref_zmp_x = 0.0f, ref_zmp_y = 0.0f, ref_zmp_th = 0.0f, prev_ref_zmp_x = 0.0f, prev_ref_zmp_y = 0.0f, prev_ref_zmp_th = 0.0f;
	float ref_zmp_x_gc = 0.0f, ref_zmp_y_gc = 0.0f, ref_zmp_th_gc = 0.0f;

    float trajectory_stride = 0.0f;
    float prev_trajectory_x = 0.0f, prev_trajectory_y = 0.0f;

    float new_cog_x = 0.0f, new_cog_y = 0.0f;
    float modi_x = 0.0f, modi_y = 0.0f;
    float total_th = 0.0f;

	float target_dx = 0.0f, target_dy = 0.0f, target_dth = 0.0f, cog_x = 0.0f, cog_y = 0.0f;
	float corr[NUM_COEF] = {0.0f};
	float foot_y = 0.0f;
	int end_time = 1000, last_step_start_time = 0;
	float disp_x = 0.0f, disp_y = 0.0f, step_dx = 0.0f, step_dy = 0.0f;
	int dir = 1;
	int sampling_num_half_cycle = (WALKING_HALF_CYCLE + SAMPLING_TIME / 2) / SAMPLING_TIME;

    std::vector<std::pair<double,double>> xyth_pts_refZMP;
    std::vector<std::pair<double,double>> xyth_pts_COG;
    std::vector<std::pair<double,double>> xyth_pts_trajectory;
    std::vector<std::pair<double,double>> xyth_cog_start_point;
    std::vector<std::pair<double,double>> modi_cog;
    std::vector<std::pair<double,double>> preview_control_zmp;
    std::vector<std::pair<double,double>> preview_control_cog;

    std::ifstream ifs;
    std::string prev_buf, prev_str, buf, str;
    std::string data_name = "45deg.csv";
    std::string prev_data = "45_prev.csv";
    int count = 0, prev_count = 0;

    float t_cog_x = 0.0f, t_cog_y = 0.0f;
    float swap_trajectory_x = 0.0f, swap_trajectory_y = 0.0f;
    float new_current_x = 0.0f, new_current_y = 0.0f, new_th = 0.0f;

//get preview_control_zmp&cog
    int res;
    prev_com = com0;
    prev_com += to_string(target[0].x, 3) + " ";
    prev_com += to_string(target[0].y, 3) + " ";
    prev_com += to_string(rad2deg(target[0].th), 1) + " ";
    prev_com += "0.0 0.0 0.0 0.0 0 0 ";
    prev_com += prev_data;
    std::cerr <<  prev_com << std::endl<< std::endl;
    const char *command = prev_com.c_str();
    res = system(command);
    std::cout << WIFEXITED(res) << "," << WEXITSTATUS(res) << std::endl;

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
    //#load preview_control_cog_data
    for(int i=0;i<prev_count;i++){
        if(ifs && std::getline(ifs, prev_buf)){
            std::istringstream stream(prev_buf);
            if(i != 0){
                for(int j=0; j<11; j++){
                    std::string num;
                    std::getline(stream, num, ' ');
                    cog_data[j] = std::atof(num.c_str());
                }
            }
        }
        preview_control_zmp.push_back(std::make_pair(cog_data[2], cog_data[5] ));
        preview_control_cog.push_back(std::make_pair(cog_data[3], cog_data[6] ));
    }
    ifs.close();
// load coefficient
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

	for(int i = 0; i < end_time; i ++){
		float t = SAMPLING_TIME * i;
		if ((i%sampling_num_half_cycle  == 0 && walking < 2) ||
			(i%(sampling_num_half_cycle/2) == 0 && walking == 2)){
			if (step < num_target - 1){
				if (t > target[step + 1].time) step ++;
			}
			com = com0;
			if (walking < 2){
				foot_y = FOOT_WIDTH * ((walking == 0) ? 0 : ((support_foot == 1) ? -1 : 1));
				target_dx = target[step].x - ref_zmp_x_gc - ref_zmp_x;
                if(target[step].th == 0){
				    target_dy = target[step].y - ref_zmp_y_gc - ref_zmp_y - foot_y;
                }else{
                    target_dy = target[step].y;
                }
                target_dth= target[step].th- ref_zmp_th_gc - ref_zmp_th;//############

                prev_trajectory_x  = - (foot_y)*sin( ref_zmp_th ) + ref_zmp_x;
                prev_trajectory_y  =   (foot_y)*cos( ref_zmp_th ) + ref_zmp_y;

                new_current_x = current_x - prev_trajectory_x - ref_zmp_x_gc;
                new_current_y = current_y - prev_trajectory_y - ref_zmp_y_gc;

                cog_x   = (new_current_x)*cos(-ref_zmp_th) - (new_current_y)*sin(-ref_zmp_th);
                cog_y   = (new_current_x)*sin(-ref_zmp_th) + (new_current_y)*cos(-ref_zmp_th);
                total_th += ref_zmp_th;
/*
                std::cout << "time: " << i*0.01 << std::endl;
                std::cout << "th  : " << rad2deg(th) << std::endl;
                std::cout << "ref_zmp_th_gc"<< rad2deg(ref_zmp_th_gc) << std::endl;
                std::cout << "prev_trajectory  "<< prev_trajectory_x <<","<<prev_trajectory_y<<std::endl;
                std::cout <<"before_end point  "<< current_x <<","<< current_y <<std::endl;
                std::cout <<"sride  "<< new_current_x <<","<< new_current_y <<std::endl;
                std::cout <<"sride_zmp  "<< new_zmp_x <<","<< new_zmp_y <<std::endl;
                std::cout <<"next_zmp  "<< to_string(set_zmp_x,3) <<","<< to_string(set_zmp_y,3) <<std::endl;
                std::cout <<"new_cog   "<< new_cog_x + prev_trajectory_x <<","<< new_cog_y + prev_trajectory_y<<std::endl;
*/

			} else foot_y = 0;
			com += to_string(target_dx, 3) + " ";
			com += to_string(target_dy, 3) + " ";
            com += to_string(rad2deg(target_dth), 3) + " ";
			com += to_string(vel_x, 4) + " ";
			com += to_string(vel_y, 4) + " ";
			com += to_string(cog_x, 4) + " ";
			com += to_string(cog_y, 4) + " ";
			com += std::to_string(support_foot) + " ";

            if (walking == 1 && (fabs(target[step].th)!=0) && fabs(rad2deg(target_dth)) < 0.05f ){
				walking = 2;
				end_time = i + sampling_num_half_cycle * 2;
				last_step_start_time = i;
            } else if (walking == 1 && fabs(target_dx) < 0.01f && fabs(target_dy) < 0.01f){
                walking = 2;
                end_time = i + sampling_num_half_cycle * 2;
                last_step_start_time = i;
			} else if (walking == 2) walking = 3;
			com += std::to_string(walking) + " ";
//			com += "temp.csv";
#if 1
			std::cerr <<  com << " test.csv" << std::endl<< std::endl;
#endif
			if ((fp = popen(com.c_str(), "r")) == NULL){
				std::cerr <<  "error" << std::endl;
				exit(-1);
			}
			for(int j = 0; j < NUM_COEF; j ++){
				char str[256], *ptr;
				fgets(str, 256, fp);
				if (feof(fp)) break;
				ptr=strchr(str, '\n');
				if (ptr != NULL) *ptr = '\0';
				corr[j] = atof(str);
#if 0
				printf("%s\n", str);
#endif
			}
//check start
			if (walking < 3){

				prev_ref_zmp_x = ref_zmp_x;
				prev_ref_zmp_y = ref_zmp_y;
                prev_ref_zmp_th= ref_zmp_th;

				ref_zmp_x = corr[0];//
				ref_zmp_y = corr[1];
                ref_zmp_th= corr[2];
#if 1
                //local pos STEP 1 & 2
                prev_trajectory_x  = - (foot_y)*sin( prev_ref_zmp_th ) + prev_ref_zmp_x;
                prev_trajectory_y  =   (foot_y)*cos( prev_ref_zmp_th ) + prev_ref_zmp_y;
                trajectory_stride  = prev_trajectory_x*cos(-prev_ref_zmp_th) - (prev_trajectory_y*sin(-prev_ref_zmp_th));

                //STEP 3
                ref_zmp_th_gc += prev_ref_zmp_th;//累積旋回角 9, 18, 27 ....
                ref_zmp_x_gc  += trajectory_stride*cos(ref_zmp_th_gc);
                ref_zmp_y_gc  += trajectory_stride*sin(ref_zmp_th_gc) + ((prev_ref_zmp_th != 0.0) ? 0.0f : (prev_ref_zmp_y + foot_y));//旋回しながら横移動は考えないものとする
                xyth_pts_trajectory.push_back(std::make_pair(ref_zmp_x_gc, ref_zmp_y_gc ));

#endif
				step_dx = (walking == 1) ? ref_zmp_x	 : 0;
				step_dy = (walking == 1) ? ref_zmp_y - foot_y : 0;
			}
//			std::cout << "step_dy: " << step_dy << std::endl;
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
            corr[3] += ref_zmp_x_gc;
            corr[9] += ref_zmp_y_gc;

			if (walking < 2){
				vel_x = vel_y = 0.0f;
				for(int j = 1; j < 6; j ++){
					vel_x += j * corr[j+3] * std::pow(WALKING_HALF_CYCLE, j - 1);
					vel_y += j * corr[j+9] * std::pow(WALKING_HALF_CYCLE, j - 1);
				}
				if (walking == 1) support_foot ^= 1;
			}
		 	if (walking == 0) walking = 1;

		}//34, 68, 102, 136 , 170

		float dt = (walking < 2) ? (i % sampling_num_half_cycle) * SAMPLING_TIME : ((i - last_step_start_time) % (sampling_num_half_cycle * 2)) * SAMPLING_TIME;
		current_x = current_y = 0.0f;
		for(int j = 0; j < 6; j ++){
			current_x += corr[j+3] * std::pow(dt, j);
			current_y += corr[j+9] * std::pow(dt, j);
		}

        //軌道接続
        modi_x = (current_x-ref_zmp_x_gc)*cos(total_th) - (current_y-ref_zmp_y_gc)*sin(total_th)+ref_zmp_x_gc;//各歩毎でのtrajectory_pointを中心に旋回角分回転
        modi_y = (current_x-ref_zmp_x_gc)*sin(total_th) + (current_y-ref_zmp_y_gc)*cos(total_th)+ref_zmp_y_gc;//

		// for checking
		double v_x = 0.0, v_y = 0.0;
		for(int j = 1; j < 6; j ++){
			v_x += j * corr[j+3] * std::pow(dt, j - 1);
			v_y += j * corr[j+9] * std::pow(dt, j - 1);
		}
		double acc_x = 0.0, acc_y = 0.0;
		for(int j = 2; j < 6; j ++){
			acc_x += j * (j - 1) * corr[j+3] * std::pow(dt, j - 2);
			acc_y += j * (j - 1) * corr[j+9] * std::pow(dt, j - 2);
		}
		if (walking == 2 && (i - last_step_start_time) == 34 / 2) foot_y = 0.0f;
#if 0
		std::cout << SAMPLING_TIME * i << "," << ref_zmp_x_gc + fabs(foot_y)*cos(ref_zmp_th_gc+(-1)*((foot_y>0.00)-(foot_y<0.00))*(M_PI/2)) << "," << ref_zmp_y_gc + fabs(foot_y)*sin(ref_zmp_th_gc+(-1)*((foot_y>0.00)-(foot_y<0.00))*(M_PI/2)) << "," << modi_x << "," << modi_y << "," <<
			v_x << "," << v_y << "," << acc_x << "," << acc_y << std::endl;
#endif
#if 1
        if(i%34 == 0){
            xyth_cog_start_point.push_back(std::make_pair(modi_x, modi_y ));
        }
        xyth_pts_COG.push_back(std::make_pair(modi_x, modi_y));
        xyth_pts_refZMP.push_back(std::make_pair(ref_zmp_x_gc + fabs(foot_y)*cos(ref_zmp_th_gc+(-1)*((foot_y>0.00)-(foot_y<0.00))*(M_PI/2)), ref_zmp_y_gc + fabs(foot_y)*sin(ref_zmp_th_gc+(-1)*((foot_y>0.00)-(foot_y<0.00))*(M_PI/2))));

#endif
	}

// plot
#if 1
    FILE *gp = popen("gnuplot -persist\n", "w");
    fprintf(gp, "set title  \"Check Coefficient\"\n");
    fprintf(gp, "set xlabel \"x [m]\"\n");
    //fprintf(gp, "set xrange[-0.05:0.3]\n");
    fprintf(gp, "set ylabel \"y [m]\"\n");
    //fprintf(gp, "set yrange[-0.05:0.2]\n");
    fprintf(gp, "set size ratio -1\n");
    fprintf(gp, "set grid \n");
    fprintf(gp, "set key left top \n");
    fprintf(gp, "plot '-' with lines lw 3 lt 2 title \"RefZMP\",'-' with lines lw 3 lt 7 title \"COM\",'-' with points lw 2 lt 1 pt 7 title \"Foot planner trajectory\",'-' with lines lw 2 lt 6 title \"TJY\",'-' with points lw 3 lt 8 pt 7 title \"COG  START Point\", '-' with lines lw 3 lt 9 dt 3 title \"PreviewControl COG\", \n");

    for(std::size_t i=0;i<xyth_pts_refZMP.size();i++) fprintf(gp, "%f\t%f\n", xyth_pts_refZMP[i].first, xyth_pts_refZMP[i].second); fprintf(gp, "e\n");
    for(std::size_t i=0;i<xyth_pts_COG.size();i++) fprintf(gp, "%f\t%f\n", xyth_pts_COG[i].first, xyth_pts_COG[i].second); fprintf(gp,"e\n");
    for(std::size_t i=0;i<xyth_pts_trajectory.size();i++) fprintf(gp, "%f\t%f\n", xyth_pts_trajectory[i].first, xyth_pts_trajectory[i].second); fprintf(gp,"e\n");
    for(std::size_t i=0;i<xyth_pts_trajectory.size();i++) fprintf(gp, "%f\t%f\n", xyth_pts_trajectory[i].first, xyth_pts_trajectory[i].second); fprintf(gp,"e\n");
    for(std::size_t i=0;i<xyth_cog_start_point.size();i++) fprintf(gp, "%f\t%f\n", xyth_cog_start_point[i].first, xyth_cog_start_point[i].second); fprintf(gp,"e\n");
    for(std::size_t i=0;i<preview_control_cog.size();i++) fprintf(gp, "%f\t%f\n", preview_control_cog[i].first, preview_control_cog[i].second); fprintf(gp,"e\n");

    fprintf(gp,"exit\n");
    pclose(gp);
#endif
	return 0;
}
