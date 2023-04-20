#==============================================================================
# plot_tune_3d.py
#==============================================================================
# 
# Visualization of the tunning developed as a part of the
# "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
#
# Author: Jakub Výmola (kuba.vymola@gmail.com)
# Date: 04/30/2023
#
#==============================================================================


import matplotlib.pyplot as plt
import sys
import scienceplots
import signal as sgnl
from scipy import signal

plt.style.use(['science', 'no-latex'])

sgnl.signal(sgnl.SIGINT, sgnl.SIG_DFL)

if len(sys.argv) < 2:
    print('Usage: python plot_tune_3d.py <log_file>')
    exit(1)

log_file = open(sys.argv[1], 'r')
header = log_file.readline().strip().split(',')


x = []
y = []
z = []
err = []

for line in log_file:
    values = line.strip().split(',')

    x.append(float(values[0]) if len(values) > 1 else 0)
    y.append(float(values[1]) if len(values) > 2 else 0)
    z.append(float(values[2]) if len(values) > 3 else 0)
    err.append(float(values[-1]))

ax = plt.figure(figsize=(8,6)).add_subplot(projection='3d')
ax.scatter(x, y, z, c=err, norm='log', cmap='RdYlGn_r', label='Convergence')
ax.set_xlabel(header[0])
ax.set_ylabel(header[1])
ax.set_zlabel(header[2])
ax.set_aspect('equal', adjustable='box')
plt.grid()
plt.show()

