set terminal postscript eps color enhanced "Times" 18
set output 'trajectory_x_0_19_y_0_0-x_0_29_y_0_1.eps'

set xrange [0:2.4]
set xlabel "Time {/Times:Italic t} (sec)"
set key left top
set multiplot layout 2,1
set datafile separator ","

set ylabel "Position {/Times:Italic x} (m)"
set yrange [0.0:0.3]
plot "trajectory_x_0_19_y_0_0-x_0_29_y_0_1.csv" u 1:2 w l title "Reference ZMP", \
	 "trajectory_x_0_19_y_0_0-x_0_29_y_0_1.csv" u 1:6 w l title "Preview control", \
	 "trajectory_x_0_19_y_0_0-x_0_29_y_0_1.csv" u 1:4 w l title "Proposed method"

set ylabel "Position {/Times:Italic y} (m)"
set yrange [-0.10:0.20]
plot "trajectory_x_0_19_y_0_0-x_0_29_y_0_1.csv" u 1:3 w l title "Reference ZMP", \
	 "trajectory_x_0_19_y_0_0-x_0_29_y_0_1.csv" u 1:7 w l title "Preview control", \
	 "trajectory_x_0_19_y_0_0-x_0_29_y_0_1.csv" u 1:5 w l title "Proposed method"

unset multiplot

