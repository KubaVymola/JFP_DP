import os
import signal as sgnl
import sys
import argparse
import subprocess
from scipy import signal
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import multiprocessing
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

sgnl.signal(sgnl.SIGINT, sgnl.SIG_DFL)

"""
This script runs a fully-contained automatic tunning algorithm. It is based on a stripped-down
version of a gradient descent. The initial values of the state vector variables, as well as the
resolution and limits are chosen by the user.

Simulations for each state vector neighborhood are ran in parallel and the state vector with the
lowest cost function value from the neighborhood is chosen for the next set of iteration, until
there are no better state vectors in the current neighborhood.
"""

class IterationPass:
    def __init__(self,
                pass_name: str,
                end_time: float,
                error_type: str,
                error_parameter: str,
                error_relax_time: float,
                error_initial_value: float,
                pass_method: str):
        self.state_vector_props: list[StateVectorProp] = []
        self.state_vector_initial_values: list[float] = []
        self.defines: list[str] = []
        self.define_values: list[str] = []

        self.transitions: list[Transition] = []

        self.pass_name: str = pass_name
        self.end_time: float = end_time
        self.method: str = pass_method
        self.error_type: str = error_type
        self.error_parameter: str = error_parameter
        self.error_relax_time: float = error_relax_time
        self.error_initial_value: float = error_initial_value

class Transition:
    def __init__(self, type, at_time, to, rate):
        self.type: float    = type
        self.at_time: float = at_time
        self.to: float      = to
        self.rate: float    = rate

class StateVectorProp:
    def __init__(self, name: str, step: float, min: float, max: float):
        self.name: str = name
        self.step: float = step
        self.max: float = max
        self.min: float = min

    def __repr__(self) -> str:
        return self.name


def main():
    args = parse_cli_args()

    iteration_passes = parse_config(args.config_file)
    
    os.chdir('../fdm/build')

    # ==== GENERATE DESIRED STEP RESPONSE FUNCTION ====
    # state_vector_values = [0.044, 0.011, 0.088] # ? Good result

    converged_results: list[list[float]] = []

    
    for iteration_pass in iteration_passes:
        print('==== running iteration pass', iteration_pass.pass_name, '====')
        converged_result = run_iteration_pass(iteration_pass, iteration_passes, converged_results)
        converged_results.append(converged_result)


    print('==== DONE ====')
    
    for converged_result,iteration_pass in zip(converged_results,iteration_passes):
        for val,name in zip(converged_result,iteration_pass.state_vector_props):
            print(f'{name}: {val}')


def parse_cli_args():
    parser = argparse.ArgumentParser()

    parser.add_argument('config_file', type=str, help='XML file with configuration')

    return parser.parse_args()


def generate_target_function(iteration_pass: IterationPass):
    time = [iteration_pass.error_relax_time]
    y_amp = [iteration_pass.error_initial_value]

    last_y = y_amp[0]

    lti = signal.lti([1.0], [0.5, 1.0])
    lti_x, lti_y = signal.step(lti)

    lti_size_x = 3.5

    for transition in iteration_pass.transitions:
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

    time.append(iteration_pass.end_time)
    y_amp.append(last_y)

    return time,y_amp


def parse_config(config_path) -> list[IterationPass]:
    iteration_passes = []
    
    tree = ET.parse(config_path)
    root = tree.getroot()

    for pass_el in root:
        if pass_el.tag != 'pass': continue

        pass_name = pass_el.attrib['name']
        end_time = float(pass_el.attrib['end_time']) if 'end_time' in pass_el.attrib else 30.0
        pass_method = pass_el.attrib['method'] if 'method' in pass_el.attrib else 'gd'

        error_el = pass_el.find('error')
        error_type = error_el.find('type').text.strip()
        error_parameter = error_el.find('parameter').text.strip()
        error_relax_time = float(error_el.attrib['relax_time'].strip()) if 'relax_time' in error_el.attrib else 0.0
        error_initial_value = float(error_el.attrib['initial_value'].strip()) if 'initial_value' in error_el.attrib else 0.0

        iteration_pass = IterationPass(
            pass_name,
            end_time,
            error_type,
            error_parameter,
            error_relax_time,
            error_initial_value,
            pass_method)

        for transition_el in pass_el.findall('transition'):
            transition_type = transition_el.attrib['type'] if 'type' in transition_el.attrib else 'lti'
            transition_at   = float(transition_el.find('at_time').text.strip())
            transition_to   = float(transition_el.find('to').text.strip())
            transition_rate = float(transition_el.find('rate').text.strip())

            transition = Transition(transition_type, transition_at, transition_to, transition_rate)
            iteration_pass.transitions.append(transition)

        for parameter_el in pass_el.findall('parameter'):
            parameter_name = parameter_el.attrib['name']
            parameter_min  = float(parameter_el.find('min').text.strip())
            parameter_max  = float(parameter_el.find('max').text.strip())
            parameter_step = float(parameter_el.find('step').text.strip())
            parameter_init = float(parameter_el.find('init').text.strip())
            
            state_vector_prop = StateVectorProp(parameter_name, parameter_step, parameter_min, parameter_max)
            iteration_pass.state_vector_props.append(state_vector_prop)
            iteration_pass.state_vector_initial_values.append(parameter_init)

        for define_el in pass_el.findall('define'):
            iteration_pass.defines.append(define_el.text.strip())
            iteration_pass.define_values.append(define_el.attrib['value'] if 'value' in define_el.attrib else None)

        iteration_passes.append(iteration_pass)

    return iteration_passes




def run_iteration_pass(iteration_pass: IterationPass, iteration_passes: list[IterationPass], converged_results: list[list[float]]):
    iteration_num = 0
    state_vector_values = iteration_pass.state_vector_initial_values
    target_x,target_y = generate_target_function(iteration_pass)

    # shutil.rmtree('../../assets/aircraft/quad/data_output/autotune/parallel', ignore_errors=True)
    Path('../../assets/aircraft/quad/data_output/autotune/parallel').mkdir(exist_ok=True, parents=True)
    
    shutil.rmtree(f'../../assets/aircraft/quad/data_output/autotune/{iteration_pass.pass_name}', ignore_errors=True)
    Path(f'../../assets/aircraft/quad/data_output/autotune/{iteration_pass.pass_name}').mkdir(exist_ok=True, parents=True)

    log_file = open(f'../../assets/aircraft/quad/data_output/autotune/{iteration_pass.pass_name}/log.csv', 'w')
    log_file.write(f"{','.join([prop.name for prop in iteration_pass.state_vector_props])},error\n")

    step_size = None

    while True:
        state_vector_surrounding = []
        
        if iteration_pass.method == 'gd':
            state_vector_surrounding = generate_state_surrounding(0, iteration_pass.state_vector_props, state_vector_values)
            state_vector_surrounding = optimize_surrounding(state_vector_surrounding, step_size)
        if iteration_pass.method == 'all':
            state_vector_surrounding = generate_all_combinations(iteration_pass.state_vector_props)

        # ==== RUN FDM ====

        print('starting', len(state_vector_surrounding), 'processes around vector', state_vector_values)

        # print(iteration_pass)
        # print(state_vector_surrounding)
        # print(iteration_passes)
        # print(converged_results)

        jobs = [(
            i,
            iteration_pass,
            state_vector,
            iteration_passes,
            converged_results,
            target_x,
            target_y,
            iteration_pass.end_time
        ) for i,state_vector in enumerate(state_vector_surrounding)]
        
        pool = multiprocessing.Pool(multiprocessing.cpu_count())
        return_values = pool.starmap(run_iteration_function, jobs)

        
        error_sums = [0] * len(state_vector_surrounding)

        for index,state_vector,error_sum in return_values:
            error_sums[index] = error_sum
            log_file.write(f"{','.join([str(x) for x in state_vector])},{str(error_sum)}\n")


        # Find best new vector from error_sums
        minimum_error_index = np.argmin(error_sums)

        if iteration_pass.method == 'all':
            return state_vector_surrounding[minimum_error_index]
        
        # Save previous best result
        shutil.copyfile(
            '../../assets/aircraft/quad/data_output/autotune/parallel/output0.csv',
            f'../../assets/aircraft/quad/data_output/autotune/{iteration_pass.pass_name}/iteration{iteration_num}.csv',
        )   

        if minimum_error_index == 0:
            log_file.close()
            print('converged in state vector', state_vector_surrounding[0], 'with error', error_sums[0])
            return state_vector_surrounding[0]

        print('found better vector at', state_vector_surrounding[minimum_error_index], 'with error', error_sums[minimum_error_index])
        step_size = [i - j for i,j in zip(state_vector_surrounding[minimum_error_index], state_vector_values)]
        state_vector_values = state_vector_surrounding[minimum_error_index]

        iteration_num += 1

# queue
def run_iteration_function(
        index,
        current_pass: IterationPass,
        state_vector: list[float],
        iteration_passes: list[IterationPass],
        converged_results: list[list[float]],
        target_x,
        target_y,
        end_time):
    args = [
        './fdm',
        'script/quad_script_001.xml',
        'aircraft/quad/output_def/quad_out_def.xml',
        '--root_dir=../../assets',
        '--sitl=../../fcs/build/libfcs.so',
        '--sitl_div=5',
        f'--sim_end={end_time}',
        '--batch',
        f'--output_path_override=aircraft/quad/data_output/autotune/parallel/output{index}.csv'
    ]

    # current iteration pass
    for value,vector_prop in zip(state_vector, current_pass.state_vector_props):
        args.append(f'--sitl_config={vector_prop.name}={value}')

    # current defiens
    for define,value in zip(current_pass.defines, current_pass.define_values):
        args.append(f'--sitl_config={define}' if value is None else f'--sitl_config={define}={value}')

    # previous iteration passes
    for iteration_pass,converged_result in zip(iteration_passes, converged_results):
        for value,vector_prop in zip(converged_result, iteration_pass.state_vector_props):
            args.append(f'--sitl_config={vector_prop.name}={value}')

    subprocess.run(
        args,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    output_file = open(f'../../assets/aircraft/quad/data_output/autotune/parallel/output{index}.csv')

    header = output_file.readline().strip().split(',')
    y_index = header.index(current_pass.error_parameter)


    # ==== CALCULATE ERROR FUNC ====

    error_sum = 0.0
    last_time = current_pass.error_relax_time

    
    for line in output_file:
        data = line.strip().split(',')

        curr_time = float(data[0])

        if curr_time < last_time:
            continue

        y = float(data[y_index])
        target_val = np.interp(curr_time, target_x, target_y)

        error_sum += (abs(y - target_val) ** 2) * abs(curr_time - last_time)
    
        last_time = curr_time

    return (index,state_vector,error_sum)




def generate_state_surrounding(index, state_vector_props, state_vector_values):
    if len(state_vector_props) != len(state_vector_values):
        raise Exception('state_vector_props and state_vector_values must have equal length')
    
    if index >= len(state_vector_props): return [[]]

    returned = generate_state_surrounding(index + 1, state_vector_props, state_vector_values)
    to_return = []

    for item in returned:
        to_return.append([state_vector_values[index]] + item)
        
        if state_vector_values[index] + state_vector_props[index].step <= state_vector_props[index].max:
            to_return.append([round(state_vector_values[index] + state_vector_props[index].step, 12)] + item)
        if state_vector_values[index] - state_vector_props[index].step >= state_vector_props[index].min:
            to_return.append([round(state_vector_values[index] - state_vector_props[index].step, 12)] + item)
        
    return to_return
    
def optimize_surrounding(state_vector_surrounding: list[list[float]], step_size: list[float]):
    if step_size is None: return state_vector_surrounding

    to_return = []

    to_return.append(state_vector_surrounding[0])

    for state_vector in state_vector_surrounding[1:]:
        for i,coordinate in enumerate(state_vector):
            if coordinate - state_vector_surrounding[0][i] > 0 and step_size[i] > 0:
                to_return.append(state_vector)
                break

            if coordinate - state_vector_surrounding[0][i] < 0 and step_size[i] < 0:
                to_return.append(state_vector)
                break


    return to_return


# TODO support multidimmensional state_vector
def generate_all_combinations(state_vector_props: list[StateVectorProp]):
    return [[i] for i in np.arange(state_vector_props[0].min, state_vector_props[0].max, state_vector_props[0].step)]


if __name__ == '__main__':
    main()