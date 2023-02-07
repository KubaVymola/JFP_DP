import math
import matplotlib.pyplot as plt
import sys

# acc: 7,8,9 (g)
# const_mult = 9.81
# const_col_num = 9

# gyro: 13,14,15 (deg/s)
const_mult = math.pi / 180
const_col_num = 14

# pressure: 35
# const_mult = 1
# const_col_num = 35

file = open(sys.argv[1] if len(sys.argv) >= 2 else '../../assets/aircraft/quad/data_output/quad_telem_idle_sensors.csv', 'r')

col_name = file.readline().split(',')[const_col_num]
print(f'col: {col_name}; mult: {const_mult}')

data = [float(line.split(',')[const_col_num]) * const_mult for line in file]

mean = sum(data) / len(data)
# result = sum((i - mean) ** 2 for i in data) / len(data)
result = math.sqrt(sum((i - mean) ** 2 for i in data) / len(data))

print(f'Mean is {mean:.20f}')
print(f'Variance is {result:.20f}')

plt.hist(data)
plt.xlabel('Measurement')
plt.ylabel('Frequency')
plt.grid()
plt.show()
