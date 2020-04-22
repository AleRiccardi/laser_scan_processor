import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    proj=[12.824659, 2.960805, '.g']
    door=[(6.707997, 6.528371), (1.439422, 0.442040)  , 'r']

    plt.plot(*proj)
    plt.plot(*door)
    plt.show()                                              