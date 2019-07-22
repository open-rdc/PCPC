#!/bin/sh

for target_x in `seq -0.3 0.01 0.3`
do
	for target_y in `seq -0.3 0.01 0.3`
	do
		./GenerateWalkPattern $target_x $target_y 0 0 0 0 0 0 temp.csv
		echo $target_x $target_y 1>&2
	done
done

