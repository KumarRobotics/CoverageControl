set terminal pngcairo enhanced font 'Times,14' size 1024, 1024
set o "data/worldmap.png"
set palette defined ( 0 "white", 1 "blue" )
set xrange [0:1024]
set yrange [0:1024]
set autoscale cbfix
set size ratio -1
set title "World Map"
plot 'data/map.dat' matrix using ($2/2):($1/2):3 with image
