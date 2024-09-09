#!/usr/bin/python3

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.pyplot import MultipleLocator
from matplotlib.font_manager import FontProperties
import rospy
import os

if __name__ == '__main__':
    rospy.init_node('my_python_node')
    
    # 获取Launch文件中定义的参数
    path_param = rospy.get_param('save_path')
    
    rospy.loginfo(f"Retrieved parameter 'save_path' with value: {path_param}")

# degeneracy_factor_file = "/home/ji/degeneracy_factor.txt"
degeneracy_factor_list = []
with open(path_param, 'r') as file:
    for line in file:
        num = float(line.strip())
        degeneracy_factor_list.append(num)

# Subscribe to file change events
file_path = os.path.abspath(path_param)
rospy.Timer(rospy.Duration(1), lambda event: update(file_path))

plt.rcParams['animation.html'] = "jshtml"
fig = plt.figure()
ax = fig.add_subplot(1,1,1)
# ax.tick_params(axis='both', labelsize=25)
ax.tick_params(axis='both', labelsize=30)
ax.plot(degeneracy_factor_list[0])
plt.xlabel('Time(s)')
plt.ylabel('Degeneracy Factor')
plt.title('Detection Result')

def update(i):
    line.set_ydata(degeneracy_factor_list)
    return line,

ani = animation.FuncAnimation(fig, update, frames=1, interval=20, blit=True)

plt.show()
