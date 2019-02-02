#!/bin/bash

target_x=0.29
target_y=0
vel_x=0.0
vel_y=0.0
cog_x=0
cog_y=0
out_of_range=`echo -e "0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0"`

#for target_th in `seq -30 5 30`
#do
	for target_th in `seq -90.0 5.0 90.0`
	do
			para="$target_x $target_y $target_th $vel_x $vel_y $cog_x $cog_y 0 0"
			coef=`./GenerateWalkPattern $para`
			#echo "$para" 1>&2
			if [ "$coef" != "$out_of_range" ]; then
				echo $para $coef
			fi

	done
#done
  echo
    for target_th in `seq -90.0 5.0 90.0`
    do
            para="$target_x $target_y $target_th $vel_x $vel_y $cog_x $cog_y 1 0"
            coef=`./GenerateWalkPattern $para`
            #echo "$para" 1>&2
            if [ "$coef" != "$out_of_range" ]; then
                echo $para $coef
            fi

    done
