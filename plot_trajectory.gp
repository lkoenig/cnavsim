if (!exists("filename")) filename='trajectory.dat'

set xlabel "x (m)"
set ylabel "y (m)"
set grid ytics lt 0 lw 1 lc rgb "#bbbbbb"
set grid xtics lt 0 lw 1 lc rgb "#bbbbbb"
set autoscale
# set terminal postscript portrait enhanced mono dashed lw 1 'Helvetica' 14
# set output 'out.eps'
set style line 1 lt 1 lw 3 pt 3 linecolor rgb "red"

plot filename using 2:3 w lines title "trajectory"

set terminal qt 1
set xlabel "time (s)"
set ylabel "speed (m/s)"
plot filename using 1:(sqrt($8 * $8 + $9 * $9)) w lines title "speed"
