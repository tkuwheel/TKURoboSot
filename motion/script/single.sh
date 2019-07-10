#!/bin/bash

cp ../../../devel/lib/motion/*.txt ./
# rm ../../../devel/lib/motion/*.txt 

python read_motor.py Single_n.txt
