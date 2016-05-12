#!/bin/bash

dev=$(file /dev/serial/by-id/* | grep Due | sed 's/.*\///' | sed "s/'//")

echo Found /dev/$dev
./server "/dev/$dev" 
