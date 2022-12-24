import argparse
from typing import TextIO
import time
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
# data_header = []
running = 0

time_prop_name = 'Time'
time_prop_col = 0
# 1d array that contains data index for each line artist
header_items = []
data_indices = []

def main():
    global running
    
    args = parse_cli_args()
    running = int(args.running)

    file = open(args.input_file, 'r')
    parse_header(file)
    fig, axs = parse_config(args.config_file)
    

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

def parse_header(file):
    global header_items, time_prop_col, data_indices
    
    header = file.readline()
    header_items = header.split(',')


def parse_config(file_name: str):
    global lns, header_items, data_indices, time_prop_name, time_prop_col, running
    
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
            time_prop_name = child.text.strip()
            time_prop_col = header_items.index(time_prop_name)
            time_range_from = float(child.attrib['range_from'])
            time_range_to   = float(child.attrib['range_to'])


    if running > 0:
        time_range_from = -running
        time_range_to = 0


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
                # data_indices.append(int(x.attrib['col']))
                data_indices.append(header_items.index(x.text.strip()))
                lns.append(axs[pos_x][pos_y].plot([], [], label=x.text.strip(), linewidth=1.5)[0])

            axs[pos_x][pos_y].grid(True, color = 'gray', linestyle = '--', linewidth = 0.5)
            axs[pos_x][pos_y].legend()

    return fig, axs

def follow_file(file):
    global new_data, data, data_indices, current_time, running, time_prop_col
    
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
            time.sleep(0.005)
            continue
        
        if file_reset:
            data = []
            current_time = 0
            file_reset = False
        
        new_data = True
        line_data = line.split(',')

        data.append([float(x) for x in line_data])
        current_time = float(line_data[time_prop_col])

        if running > 0:
            data = [line for line in data if line[time_prop_col] - current_time > -running]


def update_plot(frame):
    global new_data, data, time_prop_col, data_indices, lns, running

    if new_data:
        x_data = [line[time_prop_col] for line in data]

        if running > 0:
            x_data = [x - current_time for x in x_data]

        for i, data_index in enumerate(data_indices):
            y_data = [line[data_index] if data_index < len(line) else 0 for line in data]
            lns[i].set_data(x_data, y_data)
        
        new_data = False

    return lns

if __name__ == '__main__':
    main()
