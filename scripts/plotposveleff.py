#!/usr/bin/env python3

import matplotlib.pyplot as plt
import re

filename = "posveleff.txt"

"""
example:
    sequence: 221317
    position: (-0.8000140190124512, -1.7999984226622523, 5.00004768371582, -1.999983926812643, 1.569953441619873, 0.10002259165048599)
    velocity: (0.0, 0.0, 0.0, -0.0, 0.0, 0.0)
    effort: (1.0285425186157227, 1.20372474193573, -0.000681392615661025, 0.48010069131851196, 0.03495015949010849, -0.023206453770399094)

    sequence: 221318
    position: (-0.8000341653823853, -1.7999707661070765, 5.000023365020752, -2.0000163517394007, 1.5699458122253418, 0.09999467432498932)
    velocity: (0.0, 0.0, 0.0, -0.0, 0.0, 0.0)
    effort: (1.0213441848754883, 1.1945040225982666, 0.015278667211532593, 0.4901139736175537, 0.04051712900400162, -0.014764881692826748)

"""

x = []
j1 = []
j2 = []
j3 = []
j4 = []
j5 = []
j6 = []

with open(filename, 'r') as f:
    lines = f.readlines()
    for line in lines:
        if "sequence: " in line:
            seqs = [int(i) for i in line.split() if i.isdigit()] # will be singleton
            x.append(seqs[0])
        elif "position: " in line:
            poses = [float(i) for i in re.findall(r'[-]?\d(?:\.\d+)?', line)]
            j1.append(poses[0])
            j2.append(poses[1])
            j3.append(poses[2])
            j4.append(poses[3])
            j5.append(poses[4])
            j6.append(poses[5])

lcv = 0
for j_rot in (j1, j2, j3, j4, j5, j6):
    lcv += 1
    plt.plot(x, j_rot)
    plt.xlabel('sequence')
    plt.ylabel('joint degree')
    plt.title("line graph of joint %d rotation" % lcv)
    plt.show()

