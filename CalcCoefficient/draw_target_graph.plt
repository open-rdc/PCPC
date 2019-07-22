set terminal postscript eps color enhanced "Times" 32
set output 'step2.eps'
unset key
set xlabel 'Target  {/Times:Italic x}_{/Times:Italic target} (m)'
set ylabel 'Coefficient {/Times:Italic a}_{{/Times:Italic x}0}' rotate by 90 offset 1,0

plot "target_x.csv" u 1:9 w l

