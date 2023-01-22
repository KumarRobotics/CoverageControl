plotname=ARG1.'.png'
dataname=ARG1.'.dat'
posfilename=ARG2
goalfilename=ARG3
MAXVAL=ARG4
MAXVAL=ARG4
RES=ARG5
RANGE=ARG6

# print "script name        : ", ARG0
# print "plot name          : ", plotname
# print "dataname           : ", dataname
# print "MAXVAL             : ", MAXVAL
# print "RES                : ", RES
# print "RANGE              : ", RANGE

set terminal pngcairo enhanced font 'Times,14' size 1024, 1024
set o plotname
set palette defined ( 0 "white", 1 "blue" )
set xrange [0:RANGE]
set yrange [0:RANGE]
set cbrange [0:MAXVAL]
set size ratio -1
plot dataname matrix using ($2*RES):($1*RES):3 with image notitle,\
posfilename with points pt 7 ps 2 lc rgb "#9467bd" notitle
# goalfilename with points pt 2 ps 1 notitle,\
# posfilename using 1:2:(sprintf("%d", $0)) with labels point pt 7 ps 1 notitle
