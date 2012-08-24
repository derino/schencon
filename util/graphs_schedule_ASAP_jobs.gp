set term post enh		 # enhanced PostScript, essentially PostScript
    	      			  # with bounding boxes
set out 'graphs_schedule_ASAP_jobs.eps'

# square is synonymous to an aspect ratio of 1;
# scale y-axis by 2, retain x-axis size
#set size square #ratio 1,2 

set style line 1 lt rgb "black" lw 3 pt 6
set style line 2 lt rgb "green" lw 3 pt 6
set style line 3 lt rgb "blue" lw 3 pt 6
set style line 4 lt rgb "yellow" lw 3 pt 6
set style line 5 lt rgb "brown" lw 3 pt 6
set style line 6 lt rgb "red" lw 3 pt 6
set style line 7 lt rgb "orange" lw 3 pt 6
set style line 8 lt rgb "pink" lw 3 pt 6
set style line 9 lt rgb "purple" lw 3 pt 6
set style line 10 lt rgb "gray" lw 3 pt 6

# time
set xrange [0:21]
set xtics 1
set pointsize 2

set key top Right noreverse enhanced autotitles
# Uncomment the following to line up the axes
#set lmargin 1
#set rmargin 1
#set bmargin 1
#set tmargin 1

# plot the third graph so that it takes a quarter of the screen
set size 1,1
set origin 0,1

set title "Load power profile for jobs with ASAP scheduling"
set ylabel "Power (kW)" offset -1,0
set xlabel "time (T = 15 mins)"

#set xtics ("d_1" 7, "d_2" 8, "d_3, d_8" 19, "d_4" 6, "d_5, d_6" 10, "d_7" 13, "d_9" 17, "d_{10}" 15)

set xrange [0:21]
set yrange [0:3]
#set grid
plot "L_1.txt" title 'J_1' with steps ls 1, "L_2.txt" title 'J_2' with steps ls 2, "L_3.txt" title 'J_3' with steps ls 3, "L_4.txt" title 'J_4' with steps ls 4, "L_5.txt" title 'J_5' with steps ls 5, "L_6.txt" title 'J_6' with steps ls 6, "L_7.txt" title 'J_7' with steps ls 7, "L_8.txt" title 'J_8' with steps ls 8, "L_9.txt" title 'J_9' with steps ls 9, "L_10.txt" title 'J_{10}' with steps ls 10

# remove all customization
reset
