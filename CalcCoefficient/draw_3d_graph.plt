set terminal postscript eps color enhanced "Times" 20
set output 'coef_x_start.eps'
set datafile separator ","
unset key
set xlabel 'COG Velocity ~{/Times:Italic x}{.8.}_{{/Times:Italic g}0} (m/sec)' offset 0,-1
set ylabel 'COG Position {/Times:Italic x}_{{/Times:Italic g}0} (m)' offset 0,-2
set zlabel 'Coefficient {/Times:Italic a}_{{/Times:Italic x}0}' rotate by 90 offset 1,0

splot "coef_x_start.csv" u 3:5:9 w l #walking_x_spd_y_0_25_graph.csv
