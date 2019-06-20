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
        for line in f.readlines():
            line = line.strip("\n,' '")
            line_list = line.split(' ')
            time.append(int(line_list[0]))
            w1.append(int(line_list[1]))
            w2.append(int(line_list[2]))
            w3.append(int(line_list[3]))
                
        return time,w1,w2,w3

def Plot_Trend(time,w1,w2,w3):

    x = np.zeros(len(time))
    total_time = 0
    for i in range(len(time)):
        total_time += time[i]
        x[i] = total_time
    
    fig, ax = plt.subplots()
    ax.plot(x,w1,'r',label='w1')
    ax.plot(x,w2,'g',label='w2')
    ax.plot(x,w3,'b',label='w3')

    ax.legend(loc = 'upper right')
    plt.xlabel('time')
    plt.ylabel('velocity')
    plt.grid()
    plt.show()
    

def main():
    time,w1,w2,w3 = Read_File(sys.argv[1])
    Plot_Trend(time,w1,w2,w3)

if __name__ == '__main__':
    main()
