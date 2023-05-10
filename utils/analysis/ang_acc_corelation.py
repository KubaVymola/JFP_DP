import matplotlib.pyplot as plt
import scienceplots
import sys

"""
Doesn't work, because motors have delay
"""

plt.style.use(['science', 'no-latex'])

start_time = 110
duration = 15

roll_rate_id = 20

last_roll_rate = None
last_time = None

roll_cmd_arr = []
roll_acc_arr = []

f = open(sys.argv[1], 'r')
f.readline()

for line in f:
    data = [float(x) for x in line.strip().split(',')]

    if data[0] < start_time: continue
    if data[0] > start_time + duration: break

    if last_roll_rate is None:
        last_time = data[0]
        last_roll_rate = data[roll_rate_id]
        continue

    dt = data[0] - last_time

    roll_acc_arr.append((data[roll_rate_id] - last_roll_rate) / dt)
    roll_cmd_arr.append(data[25])

    last_time = data[0]
    last_roll_rate = data[roll_rate_id]

f.close()

plt.figure(figsize=(8,6))
plt.scatter(roll_cmd_arr, roll_acc_arr)
plt.grid()

plt.xlabel=('telem/roll-cmd-norm')
plt.ylabel=('roll acc [deg/s^2]')

plt.show()
