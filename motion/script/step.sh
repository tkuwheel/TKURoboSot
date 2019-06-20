#!/bin/bash

cp ../../../devel/lib/motion/*.txt ./
python read_motor.py Step_record.txt
