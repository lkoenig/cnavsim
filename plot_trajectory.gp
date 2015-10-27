if (!exists("filename")) filename='trajectory.dat'

set grid ytics lt 0 lw 1 lc rgb "#bbbbbb"
set grid xtics lt 0 lw 1 lc rgb "#bbbbbb"

set autoscale
set multiplot layout 3,1 rowsfirst

set autoscale
set xlabel "x (m)"
set ylabel "y (m)"
set style line 1 lt 1 lw 3 pt 3 linecolor rgb "red"
plot filename using 3:2 w lines title "trajectory"

set autoscale
set xlabel "time (s)"
set ylabel "heading (deg)"
set style line 1 lt 1 lw 3 pt 3 linecolor rgb "red"
plot filename using 1:(ceil($4 / pi * 180)%360) w lines title "heading"

set autoscale
set xlabel "time (s)"
set ylabel "speed (m/s)"
set style line 1 lt 1 lw 3 pt 3 linecolor rgb "red"
plot filename using 1:(sqrt($5 * $5 + $6 * $6)) w lines title "speed"
unset multiplot