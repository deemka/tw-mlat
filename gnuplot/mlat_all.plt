#!/urs/bin/gnuplot
set grid;
set size square 1,1; 
set xrange [-500:1200]; 
set yrange [-500:1200]; 
plot "/tmp/mlat_s0.txt" w circles
#, "/tmp/mlat_s1.txt" w circles, "/tmp/mlat_s3.txt" w circles, "/tmp/mlat_s4.txt" w circles
pause .2
reread
