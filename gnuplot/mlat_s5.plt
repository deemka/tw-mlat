#!/urs/bin/gnuplot
set term qt size 800,600
set grid;
set size square 1,1; 
set xrange [-1000:1500]; 
set yrange [-1000:1500]; 
plot "/tmp/mlat_s0.txt" w circles, '' u 1:2:(sprintf("(%d,%d)", $1, $2)) w labels offset char 4,0 title "s0"\
"/tmp/beacons.txt" u 1:2:(sprintf("B%d", $4)) w labels point pt 7 ps .75 offset char 0,1 title "beacons"
pause .2
reread
