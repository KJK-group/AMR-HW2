
set term svg
set output "plot-xyz.svg"

set terminal svg enhanced background rgb 'white' size 500, 300

set style line 1 \
    linecolor rgb '#ff0000' \
    linetype 1 linewidth 2 \
    pointtype 7 pointsize 0
set style line 2 \
    linecolor rgb '#00ff00' \
    linetype 1 linewidth 2 \
    pointtype 7 pointsize 0
set style line 3 \
    linecolor rgb '#0000ff' \
    linetype 1 linewidth 2 \
    pointtype 7 pointsize 0

set xlabel 'time (seconds)'
set ylabel 'distance (meters)'

set datafile separator ","
set grid
plot 'task3-coordinates-error.csv' u 1:2 t 'estimated x' with linespoints linestyle 1, \
     'task3-coordinates-error.csv' u 3:4 t 'estimated y' with linespoints linestyle 2, \
     'task3-coordinates-error.csv' u 5:6 t 'estimated z' with linespoints linestyle 3
