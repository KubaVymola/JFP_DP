import matplotlib.pyplot as plt
import sys
import scienceplots
import signal as sgnl
from scipy import signal

plt.style.use(['science', 'no-latex'])

sgnl.signal(sgnl.SIGINT, sgnl.SIG_DFL)

if len(sys.argv) < 2:
    print('Usage: python plot_tune_3d.py <log_file>')
    exit(1)

log_file = open(sys.argv[1], 'r')
header = log_file.readline().strip().split(',')


x = []
y = []
z = []
err = []

for line in log_file:
    values = line.strip().split(',')

    x.append(float(values[0]) if len(values) > 1 else 0)
    y.append(float(values[1]) if len(values) > 2 else 0)
    z.append(float(values[2]) if len(values) > 3 else 0)
    err.append(float(values[-1]))

ax = plt.figure(figsize=(8,6)).add_subplot(projection='3d')
ax.scatter(x, y, z, c=err, norm='log', cmap='RdYlGn_r', label='Convergence')
ax.set_xlabel(header[0])
ax.set_ylabel(header[1])
ax.set_zlabel(header[2])
ax.set_aspect('equal', adjustable='box')
plt.grid()
plt.show()



# rest_time = 10.0
# lti = signal.lti([1.0], [0.6 / 5.0, 1 / 5.0])

# time = [0.0, rest_time - 0.1]
# y_amp = [0.0, 0.0]

# l_time, l_y_amp = signal.step(lti)

# time.extend(x + rest_time for x in l_time)
# y_amp.extend(l_y_amp)

# time.extend([time[-1] + 0.1, 30.0])
# y_amp.extend([y_amp[-1], y_amp[-1]])

# plt.plot(time, y_amp)
# plt.show()

# exit(0)

