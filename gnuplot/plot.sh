#!/bin/sh

fname="/tmp/rawsig_b$1s$2.txt"
sname="/tmp/plot.plt"

set +o noclobber

cat > $sname <<EOF 
#!/urs/bin/gnuplot
#set term qt size 900,600
#set yrange [-200:200]
set grid
set size 1,1
plot "$fname" w l
pause .25
reread
#refresh
EOF

gnuplot $sname 2>/dev/null
