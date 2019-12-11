#!/bin/bash

#echo "tar_x(m) tar_y(m) tar_th(deg) vel_x(m/s) vel_y(m/s) cog_x(m) cog_y(m) L(0)R(1) STATE(0-2) ZMP_x(m) ZMP_y(m) ax0 ax1 ax2 ax3 ax4 ax5 ay0 ay1 ay2 ay3 ay4 ay5"

out_of_range=`echo -e "0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0"`

para="0.600 0.000 90.0 0.0000 0.0000 0.0000 0.0000 0 0"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.600 0.000 90.0 0.0260 -0.1205 0.0035 -0.0164 0 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.540 0.000 81.0 0.2364 0.2570 -0.0298 -0.0021 1 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.480 0.000 72.0 0.2168 -0.2375 -0.0282 0.0021 0 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.420 0.000 63.0 0.2575 0.2407 -0.0335 0.0006 1 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.360 0.000 54.0 0.2194 -0.2399 -0.0287 0.0025 0 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.300 0.000 45.0 0.2579 0.2403 -0.0336 0.0007 1 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.240 0.000 36.0 0.2195 -0.2400 -0.0287 0.0025 0 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.180 0.000 27.0 0.2578 0.2403 -0.0336 0.0007 1 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.120 0.000 18.0 0.2190 -0.2399 -0.0288 0.0026 0 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.060 0.000 9.0 0.2544 0.2417 -0.0341 0.0009 1 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.000 0.000 -0.0 0.1946 -0.2245 -0.0326 0.0050 0 2"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.000 0.000 -0.0 0.1946 -0.2245 -0.0326 0.0050 0 3"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi
