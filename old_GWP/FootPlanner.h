#ifndef _FootPlanner_H_
#define _FootPlanner_H_

#include <iostream>
#include <vector>

#include <Eigen/Core>

#include "PlanCommon.h"

using namespace std;
using namespace Eigen;

class FootPlanner{
	private:
		const double dt;
		double time, step_time;
		double zmp_correct_y, one_step_stride;
		int foot_step_count;
		double target_distance;
		FootStatus next_leg_support;
		WalkingStatus walking_status;
		Pos2D stride, target_pos, control_point, next_control_point;
	public:
		vector<Vector4d> foot_step_list;
	public:
		FootPlanner(const double _dt, const double _step_time)
			: dt(_dt), step_time(_step_time), foot_step_count(0)
		{}
		bool SetTargetPos(const double target_x, const double target_y, const double target_th, FootStatus ref_leg_sup, WalkingStatus ref_walking_status);
		bool target_pos_2_foot_step(FootStatus ref_leg_sup);
		double DistanceTargetPos(Pos2D current_pos);
		void SetFootStepParameter(const double stride_x, const double stride_y, const double stride_th, const double zmp_correct_y, const double step_time)
		{
			stride.x  = stride_x;
			stride.y  = stride_y;
			stride.th = stride_th;

			this->zmp_correct_y = zmp_correct_y;
			this->step_time		= step_time;

			one_step_stride = sqrt(pow(stride.x, 2) + pow(stride.y, 2));
		}
		void foot_step_list_clear()
		{
			foot_step_list.clear();
			foot_step_count = 0;
		}
		void start_walk(FootStatus ref_leg_sup);
		void stop_walk();
};

#endif
