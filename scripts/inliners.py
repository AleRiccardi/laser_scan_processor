import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    proj=[(1.687810, 1.711024), (-1.160000, -1.168149), 'b']
    door=[(2.033781, 1.704798), (-0.248691, -1.185887) , 'r']

    plt.plot(*proj)
    plt.plot(*door)
    plt.show()