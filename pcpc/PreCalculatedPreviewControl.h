#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "FootStepPlanner.h"
#include "PlanCommon.h"

using namespace footstep_msgs;

class PreCalculatedPreviewControl
{
    public:
        PreCalculatedPreviewControl(const double _dt, const double _gait_half_cycle, const double _preview_delay)
            : dt(_dt), gait_half_cycle(_gait_half_cycle), preview_delay(_preview_delay),
              preview_num(0), stop_time(1.0)
        {
            walk_status = StartWalking;
            support_leg = BothLeg;

            sampling_num_half_cycle = (_gait_half_cycle + _dt / 2) / _dt;

            foot_width = 0.05;

            ref_zmp[0] = ref_zmp[1] = 0.f;
            ref_zmp_gc[0] = ref_zmp_gc[1] = 0.f;

            foot_planner = new FootStepPlanner(_dt);
            foot_planner->SetFootStepParameter(0.06, 0.06, 0.0, 0.05, _gait_half_cycle);
        }
        ~PreCalculatedPreviewControl(){}
        // フットステップの保存
        void setFootStepList(std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> _foot_step_list){
            // フットステップリスト保存
            index = 1;
            foot_step_list = _foot_step_list;
         //   for(int i=0;i<foot_step_list.size();i++)
           //     std::cout << foot_step_list[i][1] << " " << foot_step_list[i][2] << std::endl;     
            // リストサイズの取得
            std::size_t step_size = foot_step_list.size();
            // 歩き終わりの時間
            end_time = foot_step_list[step_size-1][0] / dt;
            // 目標位置の取得
            target[0] = foot_step_list[step_size-1][1]; target[1] = foot_step_list[step_size-1][2];

            load_coefficient();
            calcCoefficient();

            // 歩き始めの支持脚の決定
            support_leg = (target[1] < 0) ? RightLegSup : LeftLegSup;

        //    foot_planner->SetTargetPos(target[0], target[1], 0.f, support_leg, walk_status);

            // ZMP軌道の補間
            interpolation_zmp_trajectory();
        }
        // 多項式係数の読み込み
        void load_coefficient();
        // フットステップリストからZMP軌道を生成/補間
        void interpolation_zmp_trajectory();
        void update_input();
        // 重心軌道を計算するための多項式の導出
        void calcCoefficient();
        void calc_xk(Eigen::Vector2d &com_pos, Eigen::Vector2d &com_vel, Eigen::Vector2d &com_acc);
        bool update(Eigen::Vector2d &com_pos, Eigen::Vector2d &com_vel, Eigen::Vector2d &com_acc);
    private:
        double corr[4][84];  // 多項式の係数
        double target_corr[12]; // 重心軌道計算用多項式係数
        double dt; // サンプリング周期
        double t; // 歩き始めからの時間保存
        double gait_half_cycle;
        int end_time;
        double foot_step_num;
        double stop_time;
        double target[2]; // 支持客切り替え時における次の目標位置
        double target_d[2];
        double ref_zmp[2]; // 目標ZMP
        double ref_zmp_gc[2];
        double cog[2];
        double prev_ref_zmp[2]; // 一ステップ前の目標ZMP
        double vel[2]; // ロボットの相対速度
        double current[2]; // ロボットの現在位置
        double disp[2];
        double step[2];
        double foot_y;
        double foot_width;
        int dir;
        int index;

        int sampling_num_half_cycle;
        int last_step_start_time;
        int preview_num;
        int preview_step_num;
        int preview_delay;

        WalkingStatus walk_status;
        FootStatus support_leg;

        Eigen::Vector2d com_pos, com_vel, com_acc;

        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> foot_step_list;
        std::vector<Eigen::Vector2d> refzmp_list;

        FootStepPlanner *foot_planner;
};
