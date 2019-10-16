#include "FootPlanner.h"
#include "PreviewControl.h"
#include "CurveFitting.h"

#include <string>
#include <sstream>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>

// Change for your robot and walking cycle
static const double FOOT_WIDTH			= 0.050;
static const double HEIGHT_ZC			= 0.27;
static const double ZMP_RANGE			= 0.060;
static const double WALKING_HALF_CYCLE	= 0.340;
static const double SAMPLING_TIME		= 0.010;
static const double MAX_X_STEP			= 0.060;
static const double MAX_Y_STEP			= 0.060;
static const double MAX_W_STEP          = 0.200;

double rad2deg(double radian){
    return radian * 180/M_PI;
}

double deg2rad(int deg){
    return deg * M_PI / 180;
}

int main(int argc, char *argv[])
{/*
	if (argc != 10 && argc != 11) {
		std::cout << "Generate a walking pattern by using the preview control." << std::endl <<
			"Usage: GenerateWalkPattern <target_x> <target_y> <target_th> <initial_speed_x> <initial_speed_y> " <<
			"<initial_cog_x> <initial_cog_y> <support leg 0:right 1:left> <0:start 1:walking 2:stop> [<filename>]" << std::endl <<
			"[Caution] If you do not input the <filename>, " << std::endl <<
			"1) a walking pattern for a half of the walking cycle is calculated, and" << std::endl <<
			"2) the cofficients for the Pre-Calculated Preview Control is displayed." << std::endl;
		return -1;
	}*/
	double target_x = atof(argv[1]);
	double target_y = atof(argv[2]);
    double target_th = deg2rad(atof(argv[3]));
	double speed_x = atof(argv[4]);
	double speed_y = atof(argv[5]);
	double cog_x = atof(argv[6]);
	double cog_y = atof(argv[7]);
    //double start_step = atof(argv[11]);
    //double end_step = atof(argv[12]);

	FootStatus foot_status = (atoi(argv[8]) == 1) ? RightLeg : LeftLeg;
	WalkingStatus walking_status = (atoi(argv[9]) == 1) ? Walking : Start;
	if (atoi(argv[9]) >= 2) walking_status = Stop;
	int stop_status = atoi(argv[9]) - 2;	//0:first half,1:second half
	//std::cout << "stop status:" << stop_status << std::endl;
	bool save_file = (argc == 11) ? true : false;//11->13
	std::ofstream writing_file;
	if (save_file) {
		writing_file.open(argv[10]);
		writing_file << "time(s) ref_x(m) zmp_x(m) cog_x(m) ref_y(m) zmp_y(m) cog_y(m) vel_x(m) vel_y(m) acc_x(m) acc_y(m)" << std::endl;
	}
	Vector2d com_pos, com_vel, com_acc;
	com_pos(0) = cog_x;
	com_pos(1) = cog_y;
	com_vel(0) = speed_x;
	com_vel(1) = speed_y;
	int sampling_num_half_cycle = (WALKING_HALF_CYCLE + SAMPLING_TIME / 2) / SAMPLING_TIME;

	FootPlanner plan_node(SAMPLING_TIME, WALKING_HALF_CYCLE);
	plan_node.SetFootStepParameter(MAX_X_STEP, MAX_Y_STEP, MAX_W_STEP, FOOT_WIDTH, WALKING_HALF_CYCLE);
	plan_node.SetTargetPos(target_x, target_y, target_th, foot_status, walking_status);
#if 0
	std::cout << "Results of foot steps" << std::endl;
	for (int i = 0; i < plan_node.foot_step_list.size(); i ++)
		std::cout << plan_node.foot_step_list[i][0] << ","
        << plan_node.foot_step_list[i][1] << ", "
        << plan_node.foot_step_list[i][2] << ", "
        << plan_node.foot_step_list[i][3] * 180/3.14 << "(deg) "<< ", "

        << std::endl;
#endif

#if 1
	com_acc(0) = (plan_node.foot_step_list[0][1] - cog_x) * ACCELERATION_OF_GRAVITY / HEIGHT_ZC;
	com_acc(1) = (plan_node.foot_step_list[0][2] - cog_y) * ACCELERATION_OF_GRAVITY / HEIGHT_ZC;

//	PreviewControl preview_control(SAMPLING_TIME, WALKING_HALF_CYCLE * 3);
	preview_control preview_control(SAMPLING_TIME, 1.6, HEIGHT_ZC);
	preview_control.interpolation_zmp_trajectory(plan_node.foot_step_list);
	preview_control.set_com_param(com_pos, com_vel, com_acc);

	vector<pair<double,double>> xy_pts_refZMP;
    vector<Vector3d> xyth_pts_refZMP;//add
	vector<pair<double,double>> xy_pts_ZMP;
	vector<pair<double,double>> xy_pts_COG;
	bool out_range = false;
	int calc_num = (save_file) ? 10000 : (sampling_num_half_cycle * ((walking_status == Stop) ? 2 : 1) + 2);

    //std::cout << "calc_num = " << calc_num << std::endl;

	for(int i = 0; i < calc_num; i ++){
		/* Calculation Preview Control */
		if (!preview_control.update(com_pos, com_vel, com_acc)) break;
//        std::cout << SAMPLING_TIME*i << " " << com_pos(0) << " " << com_pos(1) <<" "<< com_vel(0) << " " << com_vel(1)<< std::endl;
		/* Get Reference ZMP */
		Vector2d temp_refzmp, previous_refzmp;
        Vector3d temp_refzmp_xyth;//add
		previous_refzmp = temp_refzmp;
		preview_control.get_ref_zmp(temp_refzmp);
        preview_control.get_ref_zmp_xyth(temp_refzmp_xyth);//add
#if 0	// Get the velocity when the supporting leg is switched
		if (temp_refzmp != previous_refzmp){
			std::cout << "vel_x: " << com_vel(0) << ", vel_y: " << com_vel(1) << std::endl;
		}
#endif
		/* Get Current ZMP */
		Matrix<double,1,2> zmp;
		preview_control.output_zmp(zmp);

		float dx = zmp(0) - temp_refzmp(0), dy = zmp(1) - temp_refzmp(1);
        //std::cout << SAMPLING_TIME*i << " " << com_pos(0) << " " << com_pos(1) << std::endl;
		if (fabs(dx) > ZMP_RANGE || fabs(dy) > ZMP_RANGE) out_range = true;

		/* Wriing Gait Pattern */
		if (save_file) writing_file << SAMPLING_TIME*i << " " << temp_refzmp(0) << " " << zmp(0) << " " << com_pos(0) << " " << temp_refzmp(1) << " " << zmp(1) << " " << com_pos(1) << " " << com_vel(0) << " " << com_vel(1) << " " << com_acc(0) << " " << com_acc(1) << endl;

		xy_pts_refZMP.push_back(make_pair(temp_refzmp(0), temp_refzmp(1)));
        xyth_pts_refZMP.push_back(Vector3d(temp_refzmp_xyth(0),temp_refzmp_xyth(1),temp_refzmp_xyth(2)));
		xy_pts_ZMP.push_back(make_pair(zmp(0), zmp(1)));
		xy_pts_COG.push_back(make_pair(com_pos(0), com_pos(1)));
	}
	if (save_file) writing_file.close();


#if 1
    if (save_file){
        FILE *gp = popen("gnuplot -persist\n", "w");
/*
        fprintf(gp, "set xlabel \"x [m]\"\n");
        fprintf(gp, "set xrange[-0.1:0.5]\n");
    	fprintf(gp, "set ylabel \"y [m]\"\n");
        fprintf(gp, "set yrange[-0.1:0.18]\n");
*/
        fprintf(gp, "set size ratio -1\n");

        fprintf(gp, "set key left top \n");
        //fprintf(gp, "plot '-' with lines lw 2 lt 7 title \"COG\", '-' with lines lw 2 lt 2 title \"RefZMP\", \n");
    	fprintf(gp, "plot '-' with lines lw 4 lt 7 title \"COM\", '-' with lines lw 4 lt 2 title \"RefZMP\", \n");
        //for(std::size_t i=start_step;i<end_step;i++) fprintf(gp, "%f\t%f\n", xy_pts_COG[i].first, xy_pts_COG[i].second); fprintf(gp,"e\n");
        for(std::size_t i=0;i<xy_pts_COG.size();i++) fprintf(gp, "%f\t%f\n", xy_pts_COG[i].first, xy_pts_COG[i].second); fprintf(gp,"e\n");
    	for(std::size_t i=0;i<xy_pts_refZMP.size();i++) fprintf(gp, "%f\t%f\n", xy_pts_refZMP[i].first, xy_pts_refZMP[i].second); fprintf(gp, "e\n");
    	fprintf(gp,"exit\n");
    	pclose(gp);
    }
#endif
#if 0
    if (save_file){
        FILE *gp = popen("gnuplot -persist\n", "w");
        fprintf(gp, "set xlabel \"t [sec]\"\n");
        fprintf(gp, "set xrange[-1.0:3.5]\n");
        fprintf(gp, "set ylabel \"y [m]\"\n");
//        fprintf(gp, "set yrange[-0.06:0.16]\n"); //y
        fprintf(gp, "set yrange[-0.07:0.4]\n"); //x
    	fprintf(gp, "plot '-' with lines lw 4 lt 7 title \"COM\" \n");
//    	for(std::size_t i=0;i<xy_pts_COG.size();i++) fprintf(gp, "%f\t%f\n", i*0.01, xy_pts_COG[i].second); fprintf(gp,"e\n");
        for(std::size_t i=0;i<xy_pts_COG.size();i++) fprintf(gp, "%f\t%f\n", i*0.01, xy_pts_COG[i].first); fprintf(gp,"e\n");
    	fprintf(gp,"exit\n");
    	pclose(gp);
    }
#endif


#if 1
	if (out_range == true){
		for(int i = 0; i < 14; i ++) std::cout << "0" << std::endl;
		std::cerr << "zmp is out of range" << std::endl;
		return 0;
	}
#endif
	if (!save_file){
		int start = 0;
		int end = sampling_num_half_cycle * ((walking_status != Stop) ? 1 : 2);
		if (walking_status == Stop){
			if (stop_status == 1) start = sampling_num_half_cycle * 0.5;
			end = sampling_num_half_cycle * ((stop_status == 0) ? 0.5 : 2.0);
		}
		int num = end - start;
		//std::cout << "num:" << num << std::endl;
        //std::cout << "start, end = " << start << end << std::endl;
        //start = start + 34;
        //end = end + 34;
		VectorXd t(num), x(num), y(num);
		for(int i = 0, j = start; j < end; i ++, j ++){
            t(i) = j * SAMPLING_TIME;
			x(i) = xy_pts_COG[j].first;
			y(i) = xy_pts_COG[j].second;
            //std::cout << j * SAMPLING_TIME <<" "<< x(i) <<" "<< y(i) << std::endl;
		}

/*
		std::cout << xy_pts_refZMP[num+1].first << " ";
		std::cout << xy_pts_refZMP[num+1].second << std::endl;
*/
        std::cout << xyth_pts_refZMP[num+1](0) << " ";
        std::cout << xyth_pts_refZMP[num+1](1) << " ";
        std::cout << rad2deg(xyth_pts_refZMP[num+1](2)) << std::endl;

		CurveFitting curve_fitting(5);
        //std::cout << std::endl;
		curve_fitting.MAP(t, x, 0.0);
		std::cout << curve_fitting.GetCoefficient() << std::endl;
        //std::cout << std::endl;
		curve_fitting.MAP(t, y, 0.0);
		std::cout << curve_fitting.GetCoefficient() << std::endl;
/*
        float esp_max, esp = 0.0;
        for(int i=0;i<num;i++)
        {
            esp = fabs(curve_fitting.y(t(i)) - y(i));
            if(esp_max < esp) esp_max = esp;
        }
            std::cout << "esp_max:" << esp_max << std::endl;
*/
/*
        FILE *gp = popen("gnuplot -persist\n", "w");//-persist
        fprintf(gp, "set xlabel \"t [sec]\"\n");
        fprintf(gp, "set ylabel \"y [m]\"\n");
        //fprintf(gp, "set size ratio -1\n");
        fprintf(gp, "set colorsequence classic \n");
        fprintf(gp, "plot '-' with lines lw 3 lt 2 title \"Polynomial Function\", '-' with lines lw 3 lt 1 dt 3 title \"Preview Control\", \n");
        for(int i=0;i<num;i++) fprintf(gp, "%f\t%f\n", t(i), curve_fitting.y(t(i))); fprintf(gp, "e\n");
        for(int i=0;i<num;i++) fprintf(gp, "%f\t%f\n", t(i), y(i)); fprintf(gp,"e\n");
        fprintf(gp,"exit\n");
        pclose(gp);
*/
/*
        FILE *gp = popen("gnuplot -persist\n", "w");//-persist
        fprintf(gp, "set xlabel \"x [m]\"\n");
        fprintf(gp, "set ylabel \"y [m]\"\n");
        //fprintf(gp, "set size ratio -1\n");
        fprintf(gp, "set colorsequence classic \n");
        fprintf(gp, "plot '-' with lines lw 3 lt 1 dt 3 title \"Preview Control\", \n");
        for(int i=0;i<num;i++) fprintf(gp, "%f\t%f\n", x(i), y(i)); fprintf(gp,"e\n");
        fprintf(gp,"exit\n");
        pclose(gp);
*/
    #endif
	}

	return 0;
}
