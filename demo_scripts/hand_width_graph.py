import numpy as np
import matplotlib.pyplot as plt


# real_width = np.array([0.091285, 0.080961, 0.071536, 0.065194, 0.051337, 0.041039, 0.031831, 0.021348, 0.011484, 0.007688])
# real_width = np.array([0.091965, 0.081726, 0.071977, 0.067415, 0.051593, 0.041858, 0.031365, 0.021511, 0.011858, 0.007674])
# real_width = np.array([0.091282, 0.080977, 0.071439, 0.068798, 0.051609, 0.041778, 0.031442, 0.021475, 0.011295, 0.007688])
real_width = np.array([0.091833, 0.080722, 0.071967, 0.071921, 0.07191, 0.071803, 0.071097, 0.070812])
ideal_width = np.array([0.1 - 0.01 * (i + 1) for i in range(len(real_width))])
# ideal_width[-1] = -5.551115 * (10 ** -17)
print("ideal: ", ideal_width)

ideal_move = np.array([0.1] * len(real_width)) - ideal_width
real_move = np.array([0.1] * len(real_width)) - real_width

diff_move = ideal_move - real_move
x = [i+1 for i in range(len(diff_move))]

plt.plot(x, ideal_move, label="command value")
plt.plot(x, real_move, label="measured value")
plt.plot(x, diff_move, label="difference value")

plt.tick_params(labelbottom=False)
plt.yticks(fontsize=12)
plt.ylabel("finger closure width [m]", fontsize=18)
plt.legend(fontsize=12)
plt.subplots_adjust(left=0.15)
# plt.show()
plt.savefig("catch_balloon.png")