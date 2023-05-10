from scipy.fft import fft, fftfreq
import numpy as np
import matplotlib.pyplot as plt
import sys

import scienceplots
plt.style.use(['science', 'no-latex'])

start_time = 110
duration = 60

time_id = 0
acc_x_id = 7

acc_arr = []

f = open(sys.argv[1], 'r')
f.readline()

for line in f:
    data = [float(x) for x in line.strip().split(',')]

    if data[time_id] < start_time: continue
    if data[time_id] >= start_time + duration: break

    acc_arr.append(data[acc_x_id])
    

np_acc = np.array(acc_arr)

N = len(acc_arr)
DT = duration / N

acc_arr_fft = fft(np_acc)
acc_arr_fftfreq = fftfreq(N, DT)[:N//2]




start_time_sitl = 50
duration_sitl = 60

acc_arr_sitl = []

f = open(sys.argv[2], 'r')
f.readline()

for line in f:
    data = [float(x) for x in line.strip().split(',')]

    if data[time_id] < start_time: continue
    if data[time_id] >= start_time + duration: break

    acc_arr_sitl.append(data[acc_x_id])
    

np_acc_sitl = np.array(acc_arr_sitl)

N_sitl = len(np_acc_sitl)
DT_sitl = duration_sitl / N_sitl

print(N_sitl)
print(DT_sitl)

acc_arr_fft_sitl = fft(np_acc_sitl)
acc_arr_fftfreq_sitl = fftfreq(N_sitl, DT_sitl)[:N_sitl//2]











plt.figure(figsize=(8,6))
plt.plot(acc_arr_fftfreq, 2 / N * np.abs(acc_arr_fft[:N//2]), label='Real world')
plt.plot(acc_arr_fftfreq_sitl, 2 / N_sitl * np.abs(acc_arr_fft_sitl[:N_sitl//2]), label='SITL')
plt.yscale('log')
plt.legend()
plt.xlabel('Frequency (Hz)')
plt.ylabel('Magnitude')
plt.xlim(0, 25)
# plt.ylim(0, 0.001)
plt.grid()
plt.show()