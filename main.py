import matplotlib.pyplot as plt
import numpy as np
import math

theta = [0, 40.10, 80.21, 119.75, 159.75, 199.96, 240.07, 280.17, 320.28]
length = [100, 98, 112, 105, 98, 112, 105, 98, 112]
plt.subplot(polar=True)
plt.scatter(theta, length, s=20)
plt.show()

