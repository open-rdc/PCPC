#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>

static const double FOOT_WIDTH			= 0.050;
static const double WALKING_HALF_CYCLE	= 0.340;
static const double SAMPLING_TIME		= 0.010;
static const int NUM_COEF				= 14;

struct Target_T {
	float time;
	float x;
	float y;
};

std::string to_string(float val, int digits){
	std::ostringstream oss;
	oss << std::fixed << std::setprecision(digits) << val;
	return oss.str();
}

int main(int argc, char *argv[])
{
	struct Target_T target[10];
	if (argc < 4 || (argc-1)%3 != 0){
		std::cerr << "Generate walking patterns by the Pre-Calculated Preview Control" << std::endl <<
		"Usage: CheckCoefficient <time> <x> <y> [<time> <x> <y> ...]" << std::endl;
		exit(-1);
	}
	int num_target = (argc-1)/3;
	for(int i = 0; i < num_target; i ++){
		target[i].time = atof(argv[i * 3 + 1]);
		target[i].x    = atof(argv[i * 3 + 2]);
		target[i].y    = atof(argv[i * 3 + 3]);
	}

	std::string com;
	std::string com0 = "./GenerateWalkPattern ";
	FILE *fp;
	int support_foot = 0, step = 0, walking = 0;
	float current_x = 0.0f, current_y = 0.0f;
	float vel_x = 0.0f, vel_y = 0.0f;
	float ref_zmp_x = 0.0f, ref_zmp_y = 0.0f, prev_ref_zmp_x = 0.0f, prev_ref_zmp_y = 0.0f;
	float ref_zmp_x_gc = 0.0f, ref_zmp_y_gc = 0.0f;
	float target_dx = 0.0f, target_dy = 0.0f, cog_x = 0.0f, cog_y = 0.0f;
	float corr[NUM_COEF] = {0.0f};
	float foot_y = 0.0f;
	int end_time = 1000, last_step_start_time = 0;
	float disp_x = 0.0f, disp_y = 0.0f, step_dx = 0.0f, step_dy = 0.0f;
	int dir = 1;
	int sampling_num_half_cycle = (WALKING_HALF_CYCLE + SAMPLING_TIME / 2) / SAMPLING_TIME;

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
				target_dy = target[step].y - ref_zmp_y_gc - ref_zmp_y - foot_y;
				cog_x = current_x - ref_zmp_x_gc - ref_zmp_x;
				cog_y = current_y - ref_zmp_y_gc - ref_zmp_y - foot_y;
			} else foot_y = 0;
			com += to_string(target_dx, 2) + " ";
			com += to_string(target_dy, 2) + " ";
            //com += to_string(0, 2) + " ";//add point 2019-09
			com += to_string(vel_x, 2) + " ";
			com += to_string(vel_y, 2) + " ";
			com += to_string(cog_x, 3) + " ";
			com += to_string(cog_y, 3) + " ";
			com += std::to_string(support_foot) + " ";
			if (walking == 1 && fabs(target_dx) < 0.01f && fabs(target_dy) < 0.01f){
				walking = 2;
				end_time = i + sampling_num_half_cycle * 2;
				last_step_start_time = i;
			} else if (walking == 2) walking = 3;
			com += std::to_string(walking) + " ";
//			com += "temp.csv";
#if 0
			std::cerr << com << std::endl;
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
				disp_x = current_x - ref_zmp_x_gc - ref_zmp_x;
				disp_y = current_y - ref_zmp_y_gc - ref_zmp_y - foot_y;
				dir = (support_foot == 1) ? -1 : 1;
				prev_ref_zmp_x = ref_zmp_x;
				prev_ref_zmp_y = ref_zmp_y;
				ref_zmp_x = corr[0];
				ref_zmp_y = corr[1];
				ref_zmp_x_gc += prev_ref_zmp_x;
				ref_zmp_y_gc += (prev_ref_zmp_y + foot_y);
				step_dx = (walking == 1) ? ref_zmp_x	 : 0;
				step_dy = (walking == 1) ? ref_zmp_y - foot_y : 0;
			}
//			std::cout << "step_dy: " << step_dy << std::endl;
			if (walking == 0){
				corr[ 2] = 0.00986713 * vel_x +   1.00095 * disp_x + 0.00000150557;
				corr[ 3] =   0.960351 * vel_x + 0.0784907 * disp_x + 9.39E-05;
				corr[ 4] =   -5.88016 * vel_x -   18.2986 * disp_x + 0.0389385;
				corr[ 5] =    16.6469 * vel_x +   67.6476 * disp_x - 0.0537971;
				corr[ 6] =   -25.9567 * vel_x -   117.006 * disp_x + 0.109814;
				corr[ 7] =    18.0493 * vel_x +   85.7187 * disp_x + 0.04934;
				corr[ 8] = 0.00986713 * vel_y +   1.00095 * disp_y - 0.00000447877;
				corr[ 9] =   0.960352 * vel_y + 0.0784909 * disp_y - 0.000996702;
				corr[10] =   -5.88016 * vel_y -   18.2986 * disp_y - 0.172952;
				corr[11] =    16.6469 * vel_y +   67.6476 * disp_y + 0.255042;
				corr[12] =   -25.9567 * vel_y -   117.006 * disp_y - 0.541707;
				corr[13] =    18.0493 * vel_y +   85.7184 * disp_y - 0.171364;
			} else if (walking == 1 && fabs(target_dx) >= 0.18){
				corr[ 2] = 0.00986713 * vel_x +   1.00095 * disp_x + 0.00000683605;
				corr[ 3] =   0.960351 * vel_x + 0.0784907 * disp_x + 0.00210656;
				corr[ 4] =   -5.88016 * vel_x -   18.2986 * disp_x + 0.308368;
				corr[ 5] =    16.6469 * vel_x +   67.6476 * disp_x - 0.481904;
				corr[ 6] =   -25.9567 * vel_x -   117.006 * disp_x + 1.07158;
				corr[ 7] =    18.0493 * vel_x +   85.7187 * disp_x + 0.165453;
				corr[ 8] = 0.00986713 * vel_y +   1.00095 * disp_y + 0.000113909 * step_dy + 0.0000566483 * dir;
				corr[ 9] =   0.960352 * vel_y + 0.0784909 * disp_y +   0.0351079 * step_dy + 0.00659848   * dir;
				corr[10] =   -5.88016 * vel_y -   18.2986 * disp_y +      5.1395 * step_dy - 0.517454     * dir;
				corr[11] =    16.6469 * vel_y +   67.6476 * disp_y -     8.03195 * step_dy + 2.77058      * dir;
				corr[12] =   -25.9567 * vel_y -   117.006 * disp_y +    17.86045 * step_dy - 4.50712      * dir;
				corr[13] =    18.0493 * vel_y +   85.7184 * disp_y +     2.75671 * step_dy + 4.55405      * dir;
			} else if (walking == 1 && fabs(target_dx) >= 0.12){
				corr[ 2] =  0.00986713  * vel_x + 1.000954277 * disp_x + 1.27229E-05 * target_dx + 4.54573E-06;
				corr[ 3] =  0.960350333 * vel_x + 0.078490216 * disp_x - 0.002620714 * target_dx + 0.002578286;
				corr[ 4] = -5.880161404 * vel_x - 18.29858372 * disp_x + 0.066635714 * target_dx + 0.2963735  ;
				corr[ 5] =  16.64685105 * vel_x + 67.64731602 * disp_x + 0.060257143 * target_dx - 0.492752143;
				corr[ 6] = -25.95670404 * vel_x - 117.004961  * disp_x - 0.383785714 * target_dx + 1.140670714;
				corr[ 7] =  18.04929175 * vel_x + 85.71714892 * disp_x + 0.784332143 * target_dx + 0.024265179;
				corr[ 8] = 0.00986713 * vel_y +   1.00095 * disp_y + 0.000113909 * step_dy + 0.0000566483 * dir;
				corr[ 9] =   0.960352 * vel_y + 0.0784909 * disp_y +   0.0351079 * step_dy + 0.00659848   * dir;
				corr[10] =   -5.88016 * vel_y -   18.2986 * disp_y +      5.1395 * step_dy - 0.517454     * dir;
				corr[11] =    16.6469 * vel_y +   67.6476 * disp_y -     8.03195 * step_dy + 2.77058      * dir;
				corr[12] =   -25.9567 * vel_y -   117.006 * disp_y +    17.86045 * step_dy - 4.50712      * dir;
				corr[13] =    18.0493 * vel_y +   85.7184 * disp_y +     2.75671 * step_dy + 4.55405      * dir;
			} else if (walking == 1 && fabs(target_dx) >= 0.06){
				corr[ 2] =  0.00986713  * vel_x + 1.000954277 * disp_x + 1.23591E-05 * target_dx + 4.58932E-06;
				corr[ 3] =  0.960350333 * vel_x + 0.078490216 * disp_x + 0.004185036 * target_dx + 0.001761604;
				corr[ 4] = -5.880161404 * vel_x - 18.29858372 * disp_x + 0.582346429 * target_dx + 0.234488107;
				corr[ 5] =  16.64685105 * vel_x + 67.64731602 * disp_x - 0.956839286 * target_dx - 0.370698321;
				corr[ 6] = -25.95670404 * vel_x - 117.004961  * disp_x + 2.213864286 * target_dx + 0.828943357;
				corr[ 7] =  18.04929175 * vel_x + 85.71714892 * disp_x + 0.038196429 * target_dx + 0.113811464;
				corr[ 8] =  0.00986713  * vel_y + 1.00095     * disp_y + 0.000050603 * target_dy + 0.0000555978 * dir;
				corr[ 9] =  0.960352    * vel_y + 0.0784912   * disp_y +   0.0188648 * target_dy +   0.00693576 * dir;
				corr[10] = -5.88016     * vel_y - 18.2986     * disp_y +     2.53641 * target_dy -     0.513661 * dir;
				corr[11] =  16.6469     * vel_y + 67.6476     * disp_y -     4.04599 * target_dy +      2.74737 * dir;
				corr[12] = -25.9567     * vel_y - 117.006     * disp_y +     9.12168 * target_dy -      4.42899 * dir;
				corr[13] =  18.0493     * vel_y + 85.7184     * disp_y +    0.986705 * target_dy +      4.47628 * dir;
			} else if (walking == 1){
				corr[ 2] =  0.00986713  * vel_x + 1.000954277 * disp_x + 8.88479E-05 * target_dx - 2.2516E-22 ;
				corr[ 3] =  0.960350333 * vel_x + 0.078490216 * disp_x + 0.033544948 * target_dx - 3.581E-20  ;
				corr[ 4] = -5.880161404 * vel_x - 18.29858372 * disp_x + 4.490487582 * target_dx - 4.79369E-18;
				corr[ 5] =  16.64685105 * vel_x + 67.64731602 * disp_x - 7.135199341 * target_dx + 7.61698E-18;
				corr[ 6] = -25.95670404 * vel_x - 117.004961  * disp_x + 16.0297956  * target_dx - 3.18075E-20;
				corr[ 7] =  18.04929175 * vel_x + 85.71714892 * disp_x + 1.934754945 * target_dx - 2.06539E-18;
				corr[ 8] =  0.00986713  * vel_y + 1.00095     * disp_y + 0.000050603 * target_dy + 0.0000555978 * dir;
				corr[ 9] =  0.960352    * vel_y + 0.0784912   * disp_y +   0.0188648 * target_dy +   0.00693576 * dir;
				corr[10] = -5.88016     * vel_y - 18.2986     * disp_y +     2.53641 * target_dy -     0.513661 * dir;
				corr[11] =  16.6469     * vel_y + 67.6476     * disp_y -     4.04599 * target_dy +      2.74737 * dir;
				corr[12] = -25.9567     * vel_y - 117.006     * disp_y +     9.12168 * target_dy -      4.42899 * dir;
				corr[13] =  18.0493     * vel_y + 85.7184     * disp_y +    0.986705 * target_dy +      4.47628 * dir;
			} else if ((walking == 2)&&((i - last_step_start_time) < sampling_num_half_cycle / 2)){ // 1st half
				corr[ 2] =  0.009842334 * vel_x + 1.000792528 * disp_x + 0.000343766 * step_dx;
				corr[ 3] =  0.964866596 * vel_x + 0.110185519 * disp_x + 0.076905169 * step_dx;
				corr[ 4] = -6.028117018 * vel_x - 19.41137766 * disp_x + 13.02902198 * step_dx;
				corr[ 5] =  18.41986175 * vel_x + 82.09255844 * disp_x - 25.5853022  * step_dx;
				corr[ 6] = -34.89211035 * vel_x - 196.8393784 * disp_x + 69.92306374 * step_dx;
				corr[ 7] =  34.23064825 * vel_x + 246.0010589 * disp_x - 29.83588571 * step_dx;
				corr[ 8] =  0.009842334 * vel_y + 1.000792528 * disp_y + 0.000343766 * step_dy + 5.68033E-05 * dir;
				corr[ 9] =  0.964866596 * vel_y + 0.110185519 * disp_y + 0.076905169 * step_dy + 0.009354599 * dir;
				corr[10] = -6.028117018 * vel_y - 19.41137766 * disp_y + 13.02902198 * step_dy - 0.319110648 * dir;
				corr[11] =  18.41986175 * vel_y + 82.09255844 * disp_y - 25.5853022  * step_dy + 2.825179693 * dir;
				corr[12] = -34.89211035 * vel_y - 196.8393784 * disp_y + 69.92306374 * step_dy - 6.344316758 * dir;
				corr[13] =  34.23064825 * vel_y + 246.0010589 * disp_y - 29.83588571 * step_dy + 10.80424676 * dir;
			} else { // 2nd half
				corr[ 2] =  0.02698325  * vel_x + 1.079618597 * disp_x - 0.536945198 * step_dx;
				corr[ 3] =  0.634442754 * vel_x - 1.432506511 * disp_x + 7.118750879 * step_dx;
				corr[ 4] = -3.377784333 * vel_x - 6.561093506 * disp_x - 15.05317473 * step_dx;
				corr[ 5] =  6.899324035 * vel_x + 21.28732727 * disp_x + 18.11159451 * step_dx;
				corr[ 6] = -6.651343333 * vel_x - 23.81345498 * disp_x - 12.23722527 * step_dx;
				corr[ 7] =  2.531252298 * vel_x + 9.71487368  * disp_x + 3.636932747 * step_dx;
				corr[ 8] =  0.02698325  * vel_y + 1.079618597 * disp_y - 0.536945198 * step_dy - 0.022866289 * dir;
				corr[ 9] =  0.634442754 * vel_y - 1.432506511 * disp_y + 7.118750879 * step_dy + 0.284312242 * dir;
				corr[10] = -3.377784333 * vel_y - 6.561093506 * disp_y - 15.05317473 * step_dy - 1.080712981 * dir;
				corr[11] =  6.899324035 * vel_y + 21.28732727 * disp_y + 18.11159451 * step_dy + 1.969947393 * dir;
				corr[12] = -6.651343333 * vel_y - 23.81345498 * disp_y - 12.23722527 * step_dy - 1.802537787 * dir;
				corr[13] =  2.531252298 * vel_y + 9.71487368  * disp_y + 3.636932747 * step_dy + 0.667591974 * dir;
			}
//			corr[2] = current_x;
//			corr[8] = current_y;
			corr[2] += ref_zmp_x_gc;
			corr[8] += ref_zmp_y_gc;
			if (walking < 2){
				vel_x = vel_y = 0.0f;
				for(int j = 1; j < 6; j ++){
					vel_x += j * corr[j+2] * std::pow(WALKING_HALF_CYCLE, j - 1);
					vel_y += j * corr[j+8] * std::pow(WALKING_HALF_CYCLE, j - 1);
				}
				if (walking == 1) support_foot ^= 1;
			}
		 	if (walking == 0) walking = 1;
		}
		float dt = (walking < 2) ? (i % sampling_num_half_cycle) * SAMPLING_TIME : ((i - last_step_start_time) % (sampling_num_half_cycle * 2)) * SAMPLING_TIME;
		current_x = current_y = 0.0f;
		for(int j = 0; j < 6; j ++){
			current_x += corr[j+2] * std::pow(dt, j);
			current_y += corr[j+8] * std::pow(dt, j);
		}

		// for checking
		double v_x = 0.0, v_y = 0.0;
		for(int j = 1; j < 6; j ++){
			v_x += j * corr[j+2] * std::pow(dt, j - 1);
			v_y += j * corr[j+8] * std::pow(dt, j - 1);
		}
		double acc_x = 0.0, acc_y = 0.0;
		for(int j = 2; j < 6; j ++){
			acc_x += j * (j - 1) * corr[j+2] * std::pow(dt, j - 2);
			acc_y += j * (j - 1) * corr[j+8] * std::pow(dt, j - 2);
		}
		if (walking == 2 && (i - last_step_start_time) == 34 / 2) foot_y = 0.0f;
		std::cout << SAMPLING_TIME * i << "," << ref_zmp_x_gc << "," << ref_zmp_y_gc - foot_y << "," << current_x << "," << current_y << "," <<
			v_x << "," << v_y << "," << acc_x << "," << acc_y << std::endl;
	}
	return 0;
}
