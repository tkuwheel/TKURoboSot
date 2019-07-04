#!/usr/bin/env python3
# -*- coding: utf-8 -*-+
import sys
import numpy as np
import matplotlib.pyplot as plt



def Read_File(filename):
    with open(filename) as f:
        time = []
        w1 = []
        tar1 = []
        w1_ = []
        tar1_ = []
        err = []
        for line in f.readlines():
            line = line.strip("\n,' '")
            line_list = line.split(' ')
            time.append(int(line_list[0]))
            w1.append(float(line_list[1]))
            tar1.append(float(line_list[2]))
            w1_.append(float(line_list[3]))
            tar1_.append(float(line_list[4]))
            err.append(float(line_list[5]))
                
        return time,w1,tar1,w1_,tar1_,err

def Plot_Trend(time,w1,tar1,w1_,tar1_,err):

    x = np.zeros(len(time))
    total_time = 0
    for i in range(len(time)):
        total_time += time[i] * 0.001
        x[i] = total_time
    
    fig, ax = plt.subplots()
    ax.plot(x,w1,'y',label='real pwm',linestyle='-')
    ax.plot(x,tar1,'g',label='target pwm',linestyle='--')
    ax.plot(x,w1_,'b',label='real speed',linestyle='-')
    ax.plot(x,tar1_,'r',label='target speed',linestyle='--')
    ax.plot(x,err,'k',label='err',linestyle='-')

    ax.legend(loc = 'upper right')
    plt.xlabel('time')
    plt.ylabel('velocity')
    plt.grid()
    plt.show()
    

def main():
    Plot_Trend(*Read_File(sys.argv[1]))

if __name__ == '__main__':
    main()
