#!/usr/bin/env python3

import matplotlib.pyplot as plt
import re

filename = "posveleff.txt" # the raw static data, saved locally

"""
example of the raw static data:
    sequence: 221317
    position: (-0.8000140190124512, -1.7999984226622523, 5.00004768371582, -1.999983926812643, 1.569953441619873, 0.10002259165048599)
    velocity: (0.0, 0.0, 0.0, -0.0, 0.0, 0.0)
    effort: (1.0285425186157227, 1.20372474193573, -0.000681392615661025, 0.48010069131851196, 0.03495015949010849, -0.023206453770399094)

    sequence: 221318
    position: (-0.8000341653823853, -1.7999707661070765, 5.000023365020752, -2.0000163517394007, 1.5699458122253418, 0.09999467432498932)
    velocity: (0.0, 0.0, 0.0, -0.0, 0.0, 0.0)
    effort: (1.0213441848754883, 1.1945040225982666, 0.015278667211532593, 0.4901139736175537, 0.04051712900400162, -0.014764881692826748)

"""

x = [] # list of sequences

# lists of joint positions. 
j1 = []
j2 = []
j3 = []
j4 = []
j5 = []
j6 = []

# read through the static raw data line-by-line
with open(filename, 'r') as f:
    lines = f.readlines()
    for line in lines:
        # if the line contains "sequence", extract the number and append it to the x list
        if "sequence: " in line:
            seqs = [int(i) for i in line.split() if i.isdigit()] # will be singleton
            x.append(seqs[0])
        # if the line contains "position", extract the numbers and append them sequentially to the j lists
        elif "position: " in line:
            poses = [float(i) for i in re.findall(r'[-]?\d(?:\.\d+)?', line)] #regex pattern-matches for positive or negative float strings. \d is python for [0-9].
            j1.append(poses[0])
            j2.append(poses[1])
            j3.append(poses[2])
            j4.append(poses[3])
            j5.append(poses[4])
            j6.append(poses[5])

# make 6 seperate one-line plots and graph them all at once
lcv = 0 # lcv is a 'loop control variable'
figure, axis = plt.subplots(3, 2) # 3 rows, 2 columns
for j_rot in (j1, j2, j3, j4, j5, j6): # j_rot is one of the lists of joint rotation degrees
    j_plt = axis[int(lcv/2), (lcv%2)] #ie j1 into [0,0], j2 into [0,1], j3 into [1,0], etc
    j_plt.set_xlabel('sequence') if int(lcv/2)==2 else "" # label the x axis of the bottom two graphs with "sequence"
    j_plt.set_ylabel('joint degree') if lcv%2==0 else "" # label the left hand graphs' y-axes with "joint degree"
    lcv += 1
    j_plt.plot(x, j_rot) # plot joint rotaiton over sequence
    j_plt.set_title("line graph of joint %d rotation" % lcv)
plt.subplots_adjust(wspace=0.2, hspace=0.5)
plt.show()

# now make 1 plot with 6 lines, one for each joint
lcv = 0
for j_rot in (j1, j2, j3, j4, j5, j6):
    lcv += 1
    plt.plot(x, j_rot, label=("joint %d" % lcv))
plt.title("line graph of joint rotation")
plt.xlabel('sequence')
plt.ylabel('joint degree')
plt.legend()
plt.show()

