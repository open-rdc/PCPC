set terminal postscript eps color enhanced "Times" 20
set output 'step_th.eps'
set datafile separator ","
unset key
set xlabel 'COG Velocity ~{/Times:Italic x}{.8.}_{{/Times:Italic g}0} (m/sec)' offset 0,-1
set ylabel 'COG Position {/Times:Italic x}_{{/Times:Italic g}0} (m)' offset 0,-2
set zlabel 'Coefficient {/Times:Italic a}_{{/Times:Italic x}0}' rotate by 90 offset 1,0

splot "walking_th.csv" u 4:6:13 w l
