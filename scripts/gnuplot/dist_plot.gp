set terminal pngcairo enhanced font 'Times,14' size 1024,1024
set o "data/dists.png"
set style fill solid 0.2 noborder
set palette defined ( 0 "blue", 1 "red" )
set xrange [0:1024]
set yrange [0:1024]
set autoscale cbfix
set size ratio -1
set title "Normal Distributions" offset 0,0.1
plot 'data/dists.dat' using 1:2:($3):($4) with circles notitle palette
