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

para="0.249 0.000 36.000 0.1933 0.2908 -0.0298 -0.0021 1 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.174 0.000 27.000 0.2574 -0.2055 -0.0300 0.0040 0 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.132 0.000 18.000 0.2136 0.2699 -0.0320 0.0026 1 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.063 0.000 9.000 0.2553 -0.2143 -0.0302 0.0044 0 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.030 0.000 -0.000 0.1884 0.2401 -0.0364 -0.0011 1 2"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.030 0.000 -0.000 0.1884 0.2401 -0.0364 -0.0011 1 3"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi
