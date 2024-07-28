import matplotlib
import matplotlib.pyplot as plt
import numpy as np

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

# ROOT_PATH = '/home/dong/workspace/intersection_ECDG'
ROOT_PATH = 'D:\program\workspace\intersection_ECDG'
data_index = 1

if data_index == 1:
    # Data 1
    method1_evacuation_times = [43.6323, 209.035, 416.039, 829.554]  # Evacuation times for method 1
    method2_evacuation_times = [42.7616, 180.053, 347.014, 678.973]  # Evacuation times for method 2
    method3_evacuation_times = [36.0043, 153.637, 295.618, 573.806]  # Evacuation times for method 3
    method4_evacuation_times = [35.5668, 145.853, 275.698, 528.281]   # Evacuation times for method 4
elif data_index == 2:
    # Data 2
    method1_evacuation_times = [41.1972, 194.535, 386.207, 769.571]  # Evacuation times for method 1
    method2_evacuation_times = [40.2591, 160.914, 304.321, 588.337]  # Evacuation times for method 2
    method3_evacuation_times = [34.487, 140.935, 267.777, 514.957]  # Evacuation times for method 3
    method4_evacuation_times = [31.9797, 121.024, 223.917, 425.121]   # Evacuation times for method 4
    
    
def percentSavings(method, baseline):
    return (baseline - method) / baseline * 100.0
for i in range(4):
    print('Demand', i, 'BFST', percentSavings(method3_evacuation_times[i], method1_evacuation_times[i]), percentSavings(method3_evacuation_times[i], method2_evacuation_times[i]))
    print('Demand', i, 'BFST-DynaLane', percentSavings(method4_evacuation_times[i], method1_evacuation_times[i]), percentSavings(method4_evacuation_times[i], method2_evacuation_times[i]))
    print('Demand', i, 'BFST-DynaLane over BFST', percentSavings(method4_evacuation_times[i], method3_evacuation_times[i]))

num_vehicles = [10, 50, 100, 200]  # Number of vehicles
methods = ['FIFO', 'iDFST', 'BFST', 'BFST-DynaLane']
colors = ['#ff1f5b', '#ffc61e', '#00cd6c', '#009ade', '#00cd6c']
axis_label_font_size = 16
axis_tick_font_size = axis_label_font_size - 2
index = np.arange(len(num_vehicles))
bar_width = 0.18

plt.figure(figsize=(9, 5))

bars1 = plt.bar(index, method1_evacuation_times, bar_width, label=methods[0], color=colors[0])
bars2 = plt.bar(index + bar_width, method2_evacuation_times, bar_width, label=methods[1], color=colors[1])
bars3 = plt.bar(index + 2*bar_width, method3_evacuation_times, bar_width, label=methods[2], color=colors[2])
# bars4 = plt.bar(index + 3*bar_width, method4_evacuation_times, bar_width, label=methods[3], color=colors[3])

plt.xlabel('Number of Vehicles', fontsize=axis_label_font_size)
plt.ylabel('Evacuation Time (seconds)', fontsize=axis_label_font_size)
# plt.title('Evacuation Times vs. Number of Vehicles')
plt.xticks(index + 1.5*bar_width, num_vehicles, fontsize=axis_tick_font_size)
plt.yticks(fontsize=axis_tick_font_size)
plt.legend(fontsize=axis_tick_font_size)
plt.grid(True, linestyle='--', alpha=0.4)

# Function to annotate bars
def autolabel(bars):
    for bar in bars:
        height = bar.get_height()
        plt.annotate('{}'.format(height),
                     xy=(bar.get_x() + bar.get_width() / 2, height),
                     xytext=(0, 3),  # 3 points vertical offset
                     textcoords="offset points",
                     ha='center', va='bottom', fontsize=axis_tick_font_size)

# Annotate each bar
# autolabel(bars1)
# autolabel(bars2)
# autolabel(bars3)
# autolabel(bars4)
plt.savefig(ROOT_PATH + '/Experiments/batch_result_' + str(data_index) + '.pdf', format='pdf')

plt.tight_layout()
plt.show()
