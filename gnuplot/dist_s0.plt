#!/urs/bin/gnuplot
set grid;
plot "/tmp/distances_s0.txt" u 1 w l, '' u 2 w l, '' u 3 w l, '' u 4 w l, '' u 5 w l, '' u 6 w l ,\
  "/tmp/distances_kf_s0.txt" u 1 w l, '' u 2 w l, '' u 3 w l, '' u 4 w l, '' u 5 w l, '' u 6 w l
pause .2
reread
