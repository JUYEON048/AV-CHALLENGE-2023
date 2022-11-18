#!/usr/bin/env python
# -*- coding: utf-8 -*-
import csv
import numpy as np
import matplotlib.pyplot as plt

path_all_x = []
path_all_y = []
yaw_all = []

path = './pg_rhwnfh_ehdwpdhQkrk.csv'

files = open(path, 'r')
read = csv.reader(files)
for f in read:
    path_all_x.append(float(f[0]))
    path_all_y.append(float(f[1]))
    yaw_all.append(float(f[2]))


plt.plot(path_all_x[:], path_all_y[:], 'bo')

plt.show()