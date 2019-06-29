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
        for line in f.readlines():
            line = line.strip("\n,' '")
            line_list = line.split(' ')
            time.append(int(line_list[0]))
            w1.append(float(line_list[1]))
            tar1.append(float(line_list[2]))
                
        return time,w1,tar1

def Plot_Trend(time,w1,tar1):

    x = np.zeros(len(time))
    total_time = 0
    for i in range(len(time)):
        total_time += time[i] * 0.001
        x[i] = total_time
    
    fig, ax = plt.subplots()
    ax.plot(x,w1,'r',label='real speed',linestyle='-',marker='.')
    ax.plot(x,tar1,'b',label='target speed',linestyle='--')

    ax.legend(loc = 'upper right')
    plt.xlabel('time')
    plt.ylabel('velocity')
    plt.grid()
    plt.show()
    

def main():
    Plot_Trend(*Read_File(sys.argv[1]))

if __name__ == '__main__':
    main()
