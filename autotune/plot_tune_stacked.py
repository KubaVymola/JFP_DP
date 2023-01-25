import matplotlib.pyplot as plt
import matplotlib.cm as cm
import sys
import scienceplots
import signal as sgnl
from scipy import signal
import xml.etree.ElementTree as ET

plt.style.use(['science', 'no-latex'])

sgnl.signal(sgnl.SIGINT, sgnl.SIG_DFL)



# rest_time = 5.0
# lti = signal.lti([1.0], [0.5, 1.0])
# time, y_amp = signal.step(lti)


# plt.plot(time, y_amp)
# plt.show()

# exit(0)



class Transition:
    def __init__(self, type, at_time, to, rate):
        self.type: float    = type
        self.at_time: float = at_time
        self.to: float      = to
        self.rate: float    = rate



if len(sys.argv) < 3:
    print('Usage: python plot_tune_stacked.py <file_prefix> <config_file>')
    exit(1)

file_name_prefix = sys.argv[1]
config_file_path = sys.argv[2]

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


ax = plt.figure(figsize=(8,6)).add_subplot()

for (i,(time_list,value_list)) in enumerate(zip(time_lists, value_lists)):
    ax.plot(time_list, value_list, c=cm.RdYlGn(i / file_index))


lti_x,lti_y = generate_target_function(transitions, end_time, error_relax_time, error_initial_value)
ax.plot(lti_x, lti_y, c='b', label=f'Target {pass_name}')
ax.set_xlabel('Time [s]')
ax.set_ylabel(pass_name)
ax.legend()

plt.grid()
plt.show()
