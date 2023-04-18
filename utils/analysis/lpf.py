import numpy as np

"""
Pyton implementation of FCS's lowpass filter
"""

class lpf_t:
    cutoff_freq: float
    output: float

def lpf_init(lpf, cutoff_freq):
    lpf.cutoff_freq = cutoff_freq;
    lpf.output = -np.Infinity;

def lpf_update(lpf, input, delta_t_s):
    if lpf.output == -np.Infinity:
        lpf.output = input

    tau = 1.0 / (2.0 * np.pi * lpf.cutoff_freq);
    alpha = delta_t_s / (tau + delta_t_s);

    lpf.output = alpha * input + (1.0 - alpha) * lpf.output;

    return lpf.output;
