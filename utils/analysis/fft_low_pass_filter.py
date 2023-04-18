from scipy.fft import fft, fftfreq
import numpy as np
import matplotlib.pyplot as plt
from lpf import *

import scienceplots
plt.style.use(['science', 'no-latex'])

"""
Script that plot attenuation and phase shift of a lowpass filter. X-axis is the frequency.
Cutoff frequency is 20 Hz, therefore attenuation of 3 dB and phase shift of 45 ° is expected at
this frequency.
"""

T = 30 # end time
F = 400 # samples per sec
N = T * F
DT = 1 / F

fcutoff = 20

start = -1
end = 2

f_arr = []
db_arr = []
ang_arr = []

for fsig in np.logspace(start, end, 100):

    lpf = lpf_t()
    lpf_init(lpf, fcutoff)

    x = np.linspace(0, T, N, endpoint=True)
    y_org = np.sin(fsig * 2.0 * np.pi * x)

    y = [lpf_update(lpf, i, DT) for i in y_org]

    # plt.plot(x, y, x, y_org)

    yf = fft(y)
    yf_org = fft(y_org)
    xf = fftfreq(N, DT)[:N//2]
    # plt.plot(xf, 2 / N * np.abs(yf[:N//2]), xf, 2 / N * np.abs(yf_org[:N//2]))

    i = int(fsig * N / F)

    y_mag = 2 / N * np.abs(yf[i])
    y_mag_org = 2 / N * np.abs(yf_org[i])

    f_arr.append(fsig)
    db_arr.append(20 * np.log10(y_mag / y_mag_org))
    
    phase_diff = (np.angle(yf, deg=True) - np.angle(yf_org, deg=True))[i]
    ang_arr.append(phase_diff)

    print("freq", fsig)
    print("y_mag", y_mag)
    print("y_mag_org", y_mag_org)
    print("phase shift", phase_diff)


fig = plt.figure(figsize=(8,6))
(ax_db, ax_ang) = fig.subplots(2, sharex=True)

ax_db.plot(f_arr, db_arr)
ax_db.set_xscale('log')
ax_db.set_xlim(10 ** start, 10 ** end)
ax_db.set_ylabel('Gain [dB]')
ax_db.grid(visible=True)
ax_db.axhline(-3, color='r', linestyle='--')
ax_db.axvline(fcutoff, color='r', linestyle='--')
ax_db.text(0.15, -2.8, '-3 dB', color='r',
              rotation=0, rotation_mode='anchor', transform_rotates_text=True)
ax_db.text(fcutoff-1, -10, '20 Hz', color='r',
              rotation=90, rotation_mode='anchor', transform_rotates_text=True)


ax_ang.plot(f_arr, ang_arr)
ax_ang.set_xscale('log')
ax_ang.set_xlim(10 ** start, 10 ** end)
ax_ang.set_xlabel('Frequency [Hz]')
ax_ang.set_ylabel('Phase shift [°]')
ax_ang.grid(visible=True)
ax_ang.axhline(-45, color='r', linestyle='--')
ax_ang.axvline(fcutoff, color='r', linestyle='--')
ax_ang.text(0.15, -44, '-45°', color='r',
              rotation=0, rotation_mode='anchor', transform_rotates_text=True)
ax_ang.text(fcutoff-1, -30, '20 Hz', color='r',
              rotation=90, rotation_mode='anchor', transform_rotates_text=True)

plt.show()
