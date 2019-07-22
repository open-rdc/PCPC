#include "PreCalculatedPreviewControl.h"
#include "PlanCommon.h"

int main(int argc, char* argv[])
{
    FootStepPlanner foot_planner(0.01);
    PreCalculatedPreviewControl pre_preview_control(0.01, 0.34, 1.6);

    foot_planner.SetFootStepParameter(0.06, 0.06, 0.0, 0.05, 0.34);
    foot_planner.SetTargetPos(0.1, 0.0, 0.0, footstep_msgs::RightLegSup, footstep_msgs::StartWalking);

    pre_preview_control.setFootStepList(foot_planner.foot_step_list);

    std::vector<Eigen::Vector2d> com_pos_list;
    Eigen::Vector2d com_pos, com_vel, com_acc;
    FILE *fp = fopen("com_pos.csv", "w");
    while(1){
        if(!pre_preview_control.update(com_pos, com_vel, com_acc)) break;
        com_pos_list.push_back(com_pos);
        fprintf(fp, "%lf %lf\n", com_pos[0], com_pos[1]);
    }
    fclose(fp);

    return 0;
}