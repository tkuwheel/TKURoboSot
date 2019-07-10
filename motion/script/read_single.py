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
        cmd = []
        for line in f.readlines():
            line = line.strip("\n,' '")
            line_list = line.split(' ')
            time.append(int(line_list[0]))
            tar1.append(float(line_list[1]))
            w1.append(float(line_list[2]))
            tar1_.append(float(line_list[3]))
            w1_.append(float(line_list[4]))
            cmd.append(float(line_list[5]))
                
        return time,tar1,w1,tar1_,w1_,cmd

def Plot_Trend(time,tar1,w1,tar1_,w1_,cmd):

    x = np.zeros(len(time))
    total_time = 0
    for i in range(len(time)):
        total_time += time[i] * 0.001
        x[i] = total_time
    
    fig, ax = plt.subplots()
    ax.plot(x,tar1,'r',label='target rpm',linestyle='-')
    ax.plot(x,w1,'b',label='real rpm',linestyle='-')
    ax.plot(x,tar1_,'y',label='target pwm',linestyle='--')
    ax.plot(x,w1_,'g',label='real pwm',linestyle='-')
    ax.plot(x,cmd,'black',label='command rpm',linestyle='-.')

    ax.legend(loc = 'upper right')
    plt.xlabel('time')
    plt.ylabel('velocity')
    plt.grid()
    plt.show()
    

def main():
    Plot_Trend(*Read_File(sys.argv[1]))

if __name__ == '__main__':
    main()
