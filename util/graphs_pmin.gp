set term post enh		 # enhanced PostScript, essentially PostScript
    	      			  # with bounding boxes
set out 'graphs_pmin.eps'

# square is synonymous to an aspect ratio of 1;
# scale y-axis by 2, retain x-axis size
#set size square #ratio 1,2 

set style line 1 lt rgb "black" lw 3 pt 6
set style line 3 lt rgb "blue" lw 3 pt 6
set style line 6 lt rgb "red" lw 3 pt 6

# time
set xrange [0:21]
set xtics 1
set pointsize 2

set key top Right noreverse enhanced autotitles
# Uncomment the following to line up the axes
# set lmargin 6

# plot the first graph so that it takes a quarter of the screen
#set size 1,1
#set origin 0,3

set title "Minimum price signal"
set xlabel "time (T = 15 mins)"
set ylabel "Price (Euro)"
#set ytics ("f_{low}" 2, "f_{high}" 3)
set xrange [0:21]
set yrange [0.05:0.25]
plot "p_min.txt" title 'P_{min}' with steps ls 1 #, "p_min.txt" notitle with points


# remove all customization
reset
