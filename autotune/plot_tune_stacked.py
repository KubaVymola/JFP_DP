#==============================================================================
# plot_tune_stacked.py
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
import matplotlib.cm as cm
from matplotlib.widgets import Slider
from matplotlib.lines import Line2D
import sys
import scienceplots
import signal as sgnl
from scipy import signal
import xml.etree.ElementTree as ET

plt.style.use(['science', 'no-latex'])

sgnl.signal(sgnl.SIGINT, sgnl.SIG_DFL)

slider = False

"""
This script creates a 2D plot, where X-axis is the simulation time, and Y-axis shows the observed
variable (e.g. altitude).

Each line in the plot represents a single run of the simulation with a
given state vector variable values. The color is based simply on the number of the simulation
iteration, and has nothing to do with the cost function value.

The blue line is a target function for which the autotune tries to optimize.
"""

class Transition:
    def __init__(self, type, at_time, to, rate):
        self.type: float    = type
        self.at_time: float = at_time
        self.to: float      = to
        self.rate: float    = rate



if len(sys.argv) < 3:
    print('Usage: python plot_tune_stacked.py <file_prefix> <config_file> [--slider]')
    exit(1)

file_name_prefix = sys.argv[1]
config_file_path = sys.argv[2]

if '--slider' in sys.argv:
    slider = True

config_root = ET.parse(config_file_path).getroot()
pass_el = config_root.find('pass')
error_el = pass_el.find('error')
target_column_name = error_el.find('parameter').text.strip()
error_relax_time = float(error_el.attrib['relax_time']) if 'relax_time' in error_el.attrib else 0.0
error_initial_value = float(error_el.attrib['initial_value']) if 'initial_value' in error_el.attrib else 0.0

pass_name = pass_el.attrib['name']
end_time = float(pass_el.attrib['end_time'])
transitions = []

for transition_el in pass_el.findall('transition'):
    transition_type = transition_el.attrib['type']
    transition_at   = float(transition_el.find('at_time').text.strip())
    transition_to   = float(transition_el.find('to').text.strip())
    transition_rate = float(transition_el.find('rate').text.strip())

    transition = Transition(transition_type, transition_at, transition_to, transition_rate)
    transitions.append(transition)


file_index = 0

time_lists = []
value_lists = []

def generate_target_function(transitions, end_time, error_relax_time, error_initial_value):
    time = [error_relax_time]
    y_amp = [error_initial_value]

    last_y = y_amp[0]

    lti = signal.lti([1.0], [0.5, 1.0])
    lti_x, lti_y = signal.step(lti)

    lti_size_x = 3.5

    for transition in transitions:
        y_diff = transition.to - last_y

        if transition.type == 'linear':
            time.append(transition.at_time)
            y_amp.append(last_y)

            time.append(transition.at_time + abs(y_diff) / transition.rate)
            y_amp.append(transition.to)


        if transition.type == 'lti':
            
            for x,y in zip(lti_x, lti_y):
                time.append(transition.at_time + (x / lti_size_x) * (abs(y_diff) / transition.rate))
                y_amp.append(last_y + y * y_diff)

        
        last_y = transition.to

    time.append(end_time)
    y_amp.append(last_y)

    return time,y_amp


while True:
    try:
        log_file = open(f"{file_name_prefix}{str(file_index)}.csv")
    except:
        break

    target_column_index = log_file.readline().strip().split(',').index(target_column_name)

    time_list = []
    value_list = []

    for line in log_file:
        line_items = line.strip().split(',')
        
        time_list.append(float(line_items[0]))
        value_list.append(float(line_items[target_column_index]))

    time_lists.append(time_list)
    value_lists.append(value_list)

    file_index += 1





lti_x,lti_y = generate_target_function(transitions, end_time, error_relax_time, error_initial_value)

plots: list[Line2D] = []

fig = plt.figure(figsize=(8,6))
ax = fig.add_subplot()

iter_sel = -1

def update():
    for i,plot in enumerate(plots):
        if iter_sel < 0:
            plot.set_linestyle('-')
        elif iter_sel == i:
            plot.set_linestyle('-')
        else:
            plot.set_linestyle('None')

plt.grid()

if slider:
    fig.subplots_adjust(bottom=0.25)

    axiter = fig.add_axes([0.25, 0.1, 0.65, 0.05])
    iter_slider = Slider(
        ax=axiter,
        label='Iteration',
        valmin=0,
        valmax=len(time_lists) - 1,
        valinit=len(time_lists) - 1,
        valstep=list(range(len(time_lists)))
    )

    def slider_update(value):
        global iter_sel
        iter_sel = iter_slider.val

        update()
    

    iter_slider.on_changed(slider_update)

for (i,(time_list,value_list)) in enumerate(zip(time_lists, value_lists)):
    plots.extend(ax.plot(time_list, value_list, c=cm.RdYlGn(i / file_index)))

ax.plot(lti_x, lti_y, c='b', label=f'Target {pass_name}')
ax.set_xlabel('Time [s]')
ax.set_ylabel(pass_name)
ax.legend()

update()

plt.show()
