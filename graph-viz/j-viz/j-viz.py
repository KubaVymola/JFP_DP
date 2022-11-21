import argparse
from typing import TextIO
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from functools import partial
import signal
import xml.etree.ElementTree as ET

signal.signal(signal.SIGINT, signal.SIG_DFL)

new_data = False
lns = []
data = []
data_labels = []

def main():
    args = parse_cli_args()
    fig, axs = parse_config(args.config_file)
    
    file = open(args.input_file, 'r')
    
    data_header = parse_data_header(file)

    ani = FuncAnimation(
        fig,
        update_plot,
        partial(follow_file, file, data_header),
        blit=True,
        interval=50)
        # init_func=init_plot,

    plt.show()


def parse_cli_args():
    parser = argparse.ArgumentParser()

    parser.add_argument('config_file', type=str, help='XML file with configuration')
    parser.add_argument('input_file', type=str, help='Input file')

    return parser.parse_args()

def parse_config(file_name: str):
    global lns
    
    tree = ET.parse(file_name)
    root = tree.getroot()

    plots_x = int(root.attrib['grid_x'])
    plots_y = int(root.attrib['grid_y'])

    fig, axs = plt.subplots(plots_x, plots_y, squeeze=False, sharex=True)

    for child in root:
        if child.tag != 'lines': continue

        pos_y = int(child.attrib['pos_x'])
        pos_x = int(child.attrib['pos_y'])

        axs[pos_x][pos_y].set_xlim(float(child.attrib['x_range_min']), float(child.attrib['x_range_max']))
        axs[pos_x][pos_y].set_ylim(float(child.attrib['y_range_min']), float(child.attrib['y_range_max']))

        x_data = child.find('x_data').text
        y_data = [x.text for x in child.findall('y_data')]
        for _ in y_data:
            lns.append(axs[pos_x][pos_y].plot([], [])[0])
        
        data_labels.append([x_data] + y_data)
        data.append([[] for _ in [x_data] + y_data])

    return fig, axs

def parse_data_header(file: TextIO):
    file.seek(0)
    return file.readline().split(',')
    
def follow_file(file, data_header):
    global new_data, data, data_labels
    
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
            data = [[[] for _ in x] for x in data_labels]
            file_reset = False

        # print('data', data)
        # print('data_labels', data_labels)

        for i, labels in enumerate(data_labels):
            for j, label in enumerate(labels):
                data[i][j].append(float(line_data[data_header.index(label)]))


def update_plot(frame):
    global new_data, data, lns

    # print('update plot')
    # print(data)
    # print(data_labels)
    
    i = 0
    
    if new_data:
        for ax_data in data:
            x_data = ax_data[0]
            
            for y_data in ax_data[1:]:
                lns[i].set_data(x_data, y_data)
                i += 1
        
        new_data = False

    return lns

if __name__ == '__main__':
    main()
