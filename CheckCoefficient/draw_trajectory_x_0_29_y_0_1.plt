set terminal postscript eps color enhanced size 12.7cm,17.8cm "Times" 18
set output 'trajectory_x_0_29_y_0_1.eps'
set lmargin 10

set xrange [0:2.4]
set xlabel "Time {/Times:Italic t} (sec)"
set key left top
set multiplot layout 4,1
set datafile separator ","

set ylabel "Position {/Times:Italic x} (m)"
set yrange [0.0:0.3]
plot "trajectory_x_0_29_y_0_1.csv" u 1:4 w l title "Proposed method", \
	 "trajectory_x_0_29_y_0_1.csv" u 1:6 w l title "Preview control", \
	 "trajectory_x_0_29_y_0_1.csv" u 1:2 w l title "Reference ZMP"

set ylabel "Position {/Times:Italic y} (m)"
set yrange [-0.1:0.2]
plot "trajectory_x_0_29_y_0_1.csv" u 1:5 w l title "Proposed method", \
	 "trajectory_x_0_29_y_0_1.csv" u 1:7 w l title "Preview control", \
	 "trajectory_x_0_29_y_0_1.csv" u 1:3 w l title "Reference ZMP"

set ylabel 'Acceleration {/Times:Italic ~x{.6..}} (m/s^2)'
set yrange [-3:3]
plot "trajectory_x_0_29_y_0_1.csv" u 1:8 w l title "Proposed method", \
	 "trajectory_x_0_29_y_0_1.csv" u 1:10 w l title "Preview control"

set ylabel "Acceleration {/Times:Italic ~y{.6..}} (m/s^2)"
set yrange [-3:3]
plot "trajectory_x_0_29_y_0_1.csv" u 1:9 w l title "Proposed method", \
	 "trajectory_x_0_29_y_0_1.csv" u 1:11 w l title "Preview control"

unset multiplot

