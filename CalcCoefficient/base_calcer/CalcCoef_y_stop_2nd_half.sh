#!/bin/bash

echo "tar_x(m) tar_y(m) vel_x(m/s) vel_y(m/s) cog_x(m) cog_y(m) L(0)R(1) STATE(0-2) ZMP_x(m) ZMP_y(m) ax0 ax1 ax2 ax3 ax4 ax5 ay0 ay1 ay2 ay3 ay4 ay5"
target_x=0
vel_x=0
cog_x=0
out_of_range=`echo -e "0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0"`

vel_y=-0.25
cog_y=0
for target_y in `seq -0.06 0.01 0.06`
do
	para="$target_x $target_y $vel_x $vel_y $cog_x $cog_y 0 3"
	coef=`./GenerateWalkPattern $para`
	echo "$para" 1>&2
	if [ "$coef" != "$out_of_range" ]; then
		echo $para $coef
 	fi
done

target_y=0
cog_y=0
for vel_y in `seq -0.6 0.02 0.0`
do  
	para="$target_x $target_y $vel_x $vel_y $cog_x $cog_y 0 3"
	coef=`./GenerateWalkPattern $para`
	echo "$para" 1>&2
	if [ "$coef" != "$out_of_range" ]; then
		echo $para $coef
	fi
done

target_y=0
vel_y=-0.25
for cog_y in `seq -0.06 0.003 0.06`
do
	para="$target_x $target_y $vel_x $vel_y $cog_x $cog_y 0 3"
	coef=`./GenerateWalkPattern $para`
	echo "$para" 1>&2
	if [ "$coef" != "$out_of_range" ]; then
		echo $para $coef
	fi
done

