set terminal postscript eps color enhanced "Times" 32
set output 'target_th.eps'
unset key
set xlabel 'Target  {/Times:Italic th}_{/Times:Italic target} (deg)'
set ylabel 'Coefficient {/Times:Italic a}_{{/Times:Italic x}0}' rotate by 90 offset 1,0

plot "target_th.csv" u 3:12 w l
