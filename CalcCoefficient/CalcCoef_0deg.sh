#!/bin/bash

echo "tar_x(m) tar_y(m) tar_th(deg) vel_x(m/s) vel_y(m/s) cog_x(m) cog_y(m) L(0)R(1) STATE(0-2) ZMP_x(m) ZMP_y(m) ax0 ax1 ax2 ax3 ax4 ax5 ay0 ay1 ay2 ay3 ay4 ay5"

out_of_range=`echo -e "0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0"`

para="0.3 0 0 0.0 0.0 0.0 0.0 0 0"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $para $coef
fi

para="0.3 0 0 0.03 -0.13 0.004 -0.017 0 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $para $coef
fi

para="0.24856 0 0 0.21 0.26 -0.027 -0.003 1 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $para $coef
fi

para="0.168224 0 0 0.24 -0.24 -0.031 0.001 0 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $para $coef
fi

para="0.152915 0 0 0.24 0.24 -0.031 -0.001 1 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $para $coef
fi

para="0.052285 0 0 0.24 -0.24 -0.032 0.001 0 1"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $para $coef
fi

para="0.0 0.0 0 0.21 0.25 -0.035 -0.001 1 2"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $para $coef
fi

para="0.0 0.0 0 0.21 0.25 -0.035 -0.001 1 3"
coef=`./GenerateWalkPattern $para`
echo "$para" 1>&2
if [ "$coef" != "$out_of_range" ]; then
	echo $para $coef
fi
