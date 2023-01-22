import os
import signal as sgnl
import sys
import argparse
import subprocess
from scipy import signal
import matplotlib.pyplot as plt
import numpy as np

sgnl.signal(sgnl.SIGINT, sgnl.SIG_DFL)

end_time = 20

class StateVectorProp:
    def __init__(self, name, value_step, value_min, value_max):
        self.name = name
        # self.default_value = default_value
        self.value_step = value_step
        self.value_max = value_max
        self.value_min = value_min



def main():
    os.chdir('../fdm/build')

    # ==== GENERATE DESIRED STEP RESPONSE FUNCTION ====

    target_x,target_y = generate_lti_step()

    convergence_p = []
    convergence_i = []
    convergence_d = []
    convergence_err = []


    state_vector_props = [
        StateVectorProp('alt_sp_pid_p', 0.001, 0.01, 0.1),
        StateVectorProp('alt_sp_pid_i', 0.001, 0.01, 0.1),
        StateVectorProp('alt_sp_pid_d', 0.001, 0.01, 0.1),
    ]
    state_vector_values = [0.040, 0.009, 0.086]
    # state_vector_values = [0.044, 0.011, 0.088]
    
    while True:
        all_state_vectors = generate_state_surrounding(0, state_vector_props, state_vector_values)
        error_sums = []


        # ==== RUN FDM ====

        for state_vector in all_state_vectors:
            print('running for state vector', state_vector)

            args = [
                './fdm',
                'script/quad_script_001.xml',
                'aircraft/quad/output_def/quad_out_def.xml',
                '--root_dir=../../assets',
                '--sitl=../../fcs/build/libfcs.so',
                '--sitl_div=5',
                f'--sim_end={end_time}',
                '--batch'
            ] + [f'--sitl_config={vector_props.name}={value}' for value,vector_props in zip(state_vector, state_vector_props)]
            # print(args)
            
            subprocess.run(
                args,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

            output_file = open('../../assets/aircraft/quad/data_output/quad_out.csv')

            header = output_file.readline().strip().split(',')
            alt_index = header.index('ext/altitude-m')

            # ==== CALCULATE ERROR FUNC ====

            error_sum = 0.0
            last_time = 0.0

            data_x = []
            data_y = []

            for line in output_file:
                data = line.strip().split(',')

                curr_time = float(data[0])
                alt = float(data[alt_index])
                target_alt = np.interp(curr_time, target_x, target_y)

                data_x.append(curr_time)
                data_y.append(alt)

                error_sum += (abs(alt - target_alt) ** 2) * abs(curr_time - last_time)
            
                last_time = curr_time

            print('error_sum', error_sum)
            error_sums.append(error_sum)

        

        convergence_p.append(all_state_vectors[0][0])
        convergence_i.append(all_state_vectors[0][1])
        convergence_d.append(all_state_vectors[0][2])
        convergence_err.append(error_sums[0])

        minimum_error_index = np.argmin(error_sums)

        if minimum_error_index == 0:
            print('converged in state vector', all_state_vectors[0])
            break

        print('found better vector at', all_state_vectors[minimum_error_index], 'with error', error_sums[minimum_error_index])
        state_vector_values = all_state_vectors[minimum_error_index]


    ax = plt.figure().add_subplot(projection='3d')
    ax.scatter(convergence_p, convergence_i, convergence_d, c=plt.cm.jet(convergence_err / max(convergence_err)), label='curve in (x, y)')
    plt.show()


# def parse_cli_args():
    # parser = argparse.ArgumentParser()



def generate_lti_step():
    rest_time = 5.0

    lti = signal.lti([1.0], [0.3, 0.2])

    time = [0.0, rest_time - 0.1]
    y_amp = [0.0, 0.0]

    l_time, l_y_amp = signal.step(lti)

    time.extend(x + rest_time for x in l_time)
    y_amp.extend(l_y_amp)

    time.extend([time[-1] + 0.1, end_time])
    y_amp.extend([y_amp[-1], y_amp[-1]])

    return time,y_amp


def generate_state_surrounding(index, state_vector_props, state_vector_values):
    if len(state_vector_props) != len(state_vector_values):
        raise Exception('state_vector_props and state_vector_values must have equal length')
    
    if index >= len(state_vector_props): return [[]]

    returned = generate_state_surrounding(index + 1, state_vector_props, state_vector_values)
    to_return = []

    for item in returned:
        to_return.append([state_vector_values[index]] + item)
        
        if state_vector_values[index] + state_vector_props[index].value_step <= state_vector_props[index].value_max:
            to_return.append([state_vector_values[index] + state_vector_props[index].value_step] + item)
        if state_vector_values[index] - state_vector_props[index].value_step >= state_vector_props[index].value_min:
            to_return.append([state_vector_values[index] - state_vector_props[index].value_step] + item)
        
    return to_return
    

if __name__ == '__main__':
    main()