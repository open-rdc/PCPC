#!/bin/bash

#echo "tar_x(m) tar_y(m) tar_th(deg) vel_x(m/s) vel_y(m/s) cog_x(m) cog_y(m) L(0)R(1) STATE(0-2) ZMP_x(m) ZMP_y(m) ax0 ax1 ax2 ax3 ax4 ax5 ay0 ay1 ay2 ay3 ay4 ay5"

out_of_range=`echo -e "0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0"`

para="0.3 0 0 0.0 0.0 0.0 0.0 0 0"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.300 0.000 0.0 0.0288 -0.1255 0.0039 -0.0171 0 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.240 0.000 0.0 0.2137 0.2537 -0.0266 -0.0032 1 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.180 0.000 0.0 0.2360 -0.2396 -0.0305 0.0008 0 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.120 0.000 0.0 0.2387 0.2410 -0.0311 -0.0012 1 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.060 0.000 0.0 0.2360 -0.2437 -0.0317 0.0007 0 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.000 0.000 0.0 0.2122 0.2208 -0.0355 -0.0043 1 2"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi

para="0.000 0.000 0.0 0.2122 0.2208 -0.0355 -0.0043 1 3"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $coef
fi
