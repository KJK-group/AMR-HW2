
set term svg
set output "plot-error.svg"

set terminal svg enhanced background rgb 'white' size 500, 300

set style line 1 \
    linecolor rgb '#ff0000' \
    linetype 1 linewidth 2 \
    pointtype 7 pointsize 0

set xlabel 'time (seconds)'
set ylabel 'distance (meters)'

set yrange [-0.5:5]

set datafile separator ","
set grid

plot 'task3-coordinates-error.csv' u 7:8 t 'euclidean distance error' with linespoints linestyle 1
