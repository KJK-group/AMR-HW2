
set term svg
set output "plot-path.svg"

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

set view 45

set datafile separator ","
set grid
splot 'task3-coordinates-error.csv' u 2:4:6 t 'path' with lines
