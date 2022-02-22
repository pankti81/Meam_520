import numpy as np

from lib.calculateFK import FK
from core.interfaces import ArmController

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fk = FK()

# the dictionary below contains the data returned by calling arm.joint_limits()
limits = [
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -1.7628, 'upper': 1.7628},
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -3.0718, 'upper': -0.0698},
    {'lower': -2.8973, 'upper': 2.8973},
    {'lower': -0.0175, 'upper': 3.7525},
    {'lower': -2.8973, 'upper': 2.8973}
]

# TODO: create plot(s) which visualize the reachable workspace of the Panda arm,
# accounting for the joint limits.
#
# We've included some very basic plotting commands below, but you can find
# more functionality at https://matplotlib.org/stable/index.html

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# TODO: update this with real results
# ax.scatter(1,1,1) # plot the point (1,1,1)
space = 4
q0 = np.linspace(limits[0]['lower'], limits[0]['upper'], space)
q1 = np.linspace(limits[1]['lower'], limits[1]['upper'], space)
q2 = np.linspace(limits[2]['lower'], limits[2]['upper'], space)
q3 = np.linspace(limits[3]['lower'], limits[3]['upper'], space)
q4 = np.linspace(limits[4]['lower'], limits[4]['upper'], space)
q5 = np.linspace(limits[5]['lower'], limits[5]['upper'], space)
q6 = np.linspace(limits[6]['lower'], limits[6]['upper'], space)

for a in range(space):
    for b in range(space):
        for c in range(space):
            for d in range(space):
                for e in range(space):
                    for f in range(space):
                        for g in range(space):
                            q = np.array([q0[a], q1[b], q2[c], q3[d], q4[e], q5[f], q6[g]])
                            jointsPosition, T = fk.forward(q)
                            ax.scatter(jointsPosition[7][0], jointsPosition[7][1], jointsPosition[7][2], s=1)

plt.show()
