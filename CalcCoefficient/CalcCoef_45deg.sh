#!/bin/bash

#echo "tar_x(m) tar_y(m) tar_th(deg) vel_x(m/s) vel_y(m/s) cog_x(m) cog_y(m) L(0)R(1) STATE(0-2) ZMP_x(m) ZMP_y(m) ax0 ax1 ax2 ax3 ax4 ax5 ay0 ay1 ay2 ay3 ay4 ay5"

out_of_range=`echo -e "0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0"`

para="0.3 0 45 0.0 0.0 0.0 0.0 0 0"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.300 0.000 45.000 0.0260 -0.1205 0.0035 -0.0164 0 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.240 0.000 36.0 0.2364 0.2503 -0.0298 -0.0021 1 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.180 0.000 27.0 0.2170 -0.2313 -0.0283 0.0018 0 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.120 0.000 18.0 0.2572 0.2333 -0.0336 0.0007 1 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.060 0.000 9.0 0.2164 -0.2375 -0.0292 0.0017 0 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.000 0.000 -0.0 0.2287 0.2088 -0.0381 -0.0031 1 2"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.000 0.000 -0.0 0.2287 0.2088 -0.0381 -0.0031 1 3"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi
