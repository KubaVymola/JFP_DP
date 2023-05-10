import matplotlib.pyplot as plt
import scienceplots
import sys

plt.style.use(['science', 'no-latex'])

start_time = 110
duration = 15

roll_arr = []
pitch_arr = []
time_arr = []

f = open(sys.argv[1], 'r')
f.readline()

for line in f:
    data = [float(x) for x in line.strip().split(',')]

    if (data[0] < start_time): continue
    if (data[0] > start_time + duration): break

    time_arr.append(data[0])
    pitch_arr.append(data[17])
    roll_arr.append(data[18])

f.close()

plt.figure(figsize=(8,6))
plt.plot(time_arr, roll_arr, label="telem/roll")
plt.plot(time_arr, pitch_arr, label="telem/pitch")
plt.grid()
plt.legend()

plt.xlim(start_time, start_time + duration)
plt.xlabel("Time [s]")
plt.ylabel("Angle [deg]")

plt.show()
