#!/usr/bin/env python3
# -*- coding: utf-8 -*-+
import sys
import numpy as np
import matplotlib.pyplot as plt



def Read_File(filename):
    with open(filename) as f:
        time = []
        w1 = []
        w2 = []
        w3 = []
        tar1 = []
        tar2 = []
        tar3 = []
        for line in f.readlines():
            line = line.strip("\n,' '")
            line_list = line.split(' ')
            time.append(int(line_list[0]))
            w1.append(float(line_list[1]))
            w2.append(float(line_list[2]))
            w3.append(float(line_list[3]))
            tar1.append(float(line_list[4]))
            tar2.append(float(line_list[5]))
            tar3.append(float(line_list[6]))
                
        return time,w1,w2,w3,tar1,tar2,tar3

def Plot_Trend(time,w1,w2,w3,tar1,tar2,tar3):

    x = np.zeros(len(time))
    total_time = 0
    for i in range(len(time)):
        total_time += time[i] * 0.001
        x[i] = total_time
    
    fig, ax = plt.subplots()
    ax.plot(x,w1,'blue',label='w1',linestyle='-')
    ax.plot(x,w2,'orange',label='w2',linestyle='-')
    ax.plot(x,w3,'green',label='w3',linestyle='-')
    ax.plot(x,tar1,'red',label='target1',linestyle='--')
    ax.plot(x,tar2,'black',label='target2',linestyle='--')
    ax.plot(x,tar3,'yellow',label='target3',linestyle='--')

    ax.legend(loc = 'upper right')
    plt.xlabel('time')
    plt.ylabel('velocity')
    plt.grid()
    plt.show()
    

def main():
    Plot_Trend(*Read_File(sys.argv[1]))

if __name__ == '__main__':
    main()
