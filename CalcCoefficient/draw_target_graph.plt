set terminal postscript eps color enhanced "Times" 32
set output 'step1210.eps'
set datafile separator ","
unset key
set xlabel 'Target  {/Times:Italic deg}_{/Times:Italic target} (deg)'
set ylabel 'Coefficient {/Times:Italic a}_{{/Times:Italic x}4}' rotate by 90 offset 1,0
set pointsize 1
#plot "target_x.csv" u 1:9 lw 2 lt 7 pt 7
plot "target_th_pm90deg.csv" u 3:17 lw 2 lt 7 pt 7

