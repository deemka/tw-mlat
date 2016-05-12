#!/urs/bin/gnuplot
set term qt size 800,600
set grid;
set size square 1,1; 
set xrange [-1500:2000]; 
set yrange [-1500:2000]; 
plot "/tmp/mlat_s0.txt" w circles,\
"/tmp/beacons.txt" u 1:2:(sprintf("B%d", $4)) w labels point pt 7 ps .75 offset char 0,1 title "beacons"
pause .2
reread
