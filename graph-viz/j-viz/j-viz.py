import argparse
from typing import TextIO
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from functools import partial
import signal
import xml.etree.ElementTree as ET
import scienceplots

plt.style.use(['science', 'no-latex'])

signal.signal(signal.SIGINT, signal.SIG_DFL)

new_data = False
lns = []    

# 2d array that contains each line
data = []
# Highest time value
current_time = 0
# 1d array that contains the first line of the log
data_header = []
running = 0

time_property = 'Time'
# 1d array that contains property name for each line artist
data_properties = []

def main():
    global running
    
    args = parse_cli_args()
    running = int(args.running)

    fig, axs = parse_config(args.config_file)
    
    file = open(args.input_file, 'r')
    
    parse_data_header(file)

    ani = FuncAnimation(
        fig,
        update_plot,
        partial(follow_file, file),
        blit=True,
        interval=50)

    plt.show()


def parse_cli_args():
    parser = argparse.ArgumentParser()

    parser.add_argument('config_file', type=str, help='XML file with configuration')
    parser.add_argument('input_file', type=str, help='Input file')
    parser.add_argument('--running', action='store', nargs='?', default=0, help='With this flag only last RUNNING seconds will be plotted')

    return parser.parse_args()

def parse_config(file_name: str):
    global lns, data_properties, time_property, running
    
    tree = ET.parse(file_name)
    root = tree.getroot()

    plots_x = int(root.attrib['grid_x'])
    plots_y = int(root.attrib['grid_y'])

    fig, axs = plt.subplots(plots_y, plots_x, squeeze=False, sharex=True)

    fig.set_size_inches(10, 8)

    time_range_from = 0
    time_range_to   = 60

    for child in root:
        if child.tag == 'time':
            time_property = child.text.strip()
            time_range_from = float(child.attrib['range_from'])
            time_range_to   = float(child.attrib['range_to'])


    if running > 0:
        time_range_from = -running
        time_range_to = 0

    print(running)
    print(time_range_from)
    print(time_range_to)


    for child in root:
        if child.tag == 'plot':
            pos_y = int(child.attrib['pos_x'])
            pos_x = int(child.attrib['pos_y'])

            data_range_from = float(child.attrib['range_from'])
            data_range_to   = float(child.attrib['range_to'])

            axs[pos_x][pos_y].set_xlim(time_range_from, time_range_to)
            axs[pos_x][pos_y].set_ylim(data_range_from, data_range_to)
            
            axs[pos_x][pos_y].set_xlabel('Time [s]')
            if child.attrib['ylabel']:
                axs[pos_x][pos_y].set_ylabel(child.attrib['ylabel'])
            if child.attrib['title']:
                axs[pos_x][pos_y].set_title(child.attrib['title'])
            
            for x in child.findall('data'):
                new_data_property = x.text.strip()
                data_properties.append(new_data_property)
                lns.append(axs[pos_x][pos_y].plot([], [], label='/'.join(new_data_property.split('/')[3:]), linewidth=1.5)[0])

            axs[pos_x][pos_y].grid(True, color = 'gray', linestyle = '--', linewidth = 0.5)
            axs[pos_x][pos_y].legend()

    return fig, axs

def parse_data_header(file: TextIO):
    global data_header
    
    file.seek(0)
    data_header = [x.strip() for x in file.readline().split(',')]
    
def follow_file(file):
    global new_data, data, data_properties, current_time, running
    
    file_reset = False
    
    while True:
        line = file.readline()

        if not line:
            before_seek = file.tell()
            file.seek(0, 2)
            after_seek = file.tell()

            if after_seek < before_seek:
                file_reset = True

            if after_seek == 0:
                file.readline()

            yield True
            continue
        
        new_data = True
        line_data = line.split(',')
        
        if file_reset:
            data = []
            current_time = 0
            file_reset = False

        data.append([float(x) for x in line_data])
        current_time = float(line_data[data_header.index(time_property)])

        if running > 0:
            data = [line for line in data if line[data_header.index(time_property)] - current_time > -running]


def update_plot(frame):
    global new_data, data, data_header, time_property, data_properties, lns, running

    if new_data:
        x_data = [x[data_header.index(time_property)] for x in data]

        if running > 0:
            x_data = [x - current_time for x in x_data]

        for i, data_property in enumerate(data_properties):
            index = data_header.index(data_property)
            y_data = [y[index] for y in data]

            lns[i].set_data(x_data, y_data)
        
        new_data = False

    return lns

if __name__ == '__main__':
    main()
