#!/bin/bash

echo "tar_x(m) tar_y(m) tar_th(deg) vel_x(m/s) vel_y(m/s) cog_x(m) cog_y(m) L(0)R(1) STATE(0-2) ZMP_x(m) ZMP_y(m) ZMP_th(deg) ax0 ax1 ax2 ax3 ax4 ax5 ay0 ay1 ay2 ay3 ay4 ay5"
target_y=0
target_deg=0
vel_y=0
cog_y=0
out_of_range=`echo -e "0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0"`

target_x=0.30
target_deg=0
vel_y=-0.12
for vel_x in `seq -0.4 0.02 0.4`
do
    for cog_x in `seq -0.06 0.003 0.06`
    do
        para="$target_x $target_y $target_deg $vel_x $vel_y $cog_x $cog_y 0 1"
        coef=`./GenerateWalkPattern $para`
        echo "$para" 1>&2
        if [ "$coef" != "$out_of_range" ]; then
        	echo $para $coef
        fi
    done
    echo " "
done

for cog_x in `seq -0.06 0.003 0.06`
do
    for vel_x in `seq -0.4 0.02 0.4`
    do
        para="$target_x $target_y $target_deg $vel_x $vel_y $cog_x $cog_y 0 1"
        coef=`./GenerateWalkPattern $para`
        echo "$para" 1>&2
        if [ "$coef" != "$out_of_range" ]; then
        	echo $para $coef
        fi
    done
    echo " "
done
