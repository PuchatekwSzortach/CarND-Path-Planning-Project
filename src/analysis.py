"""
Simple track analysis
"""

import numpy as np
import matplotlib.pyplot as plt


def main():

    data_path = "./data/highway_map.csv"

    # Load x and y points
    data = np.loadtxt(data_path)[:, :2]

    x = data[:, 0]
    y = data[:, 1]

    delta_x = x[1:] - x[:-1]
    delta_y = y[1:] - y[:-1]

    distances = np.sqrt(delta_x**2 + delta_y**2)

    target_speed = 22

    print("Total distance: {} metres".format(np.sum(distances)))
    print("Min map waypoints distance: {} metres".format(np.min(distances)))
    print("Median map waypoints distance: {} metres".format(np.median(distances)))
    print("Max map waypoints distance: {} metres".format(np.max(distances)))
    print("We want to be driving roughly at {} m/s".format(target_speed))

    update_time = 0.02

    print("Car moves provided points in {} sec".format(update_time))
    print("Distance between two points should be about {}".format(target_speed * update_time))

    print("Thus we should have about {} steps between two map points".format(
        np.median(distances) / (target_speed * update_time)))

    print("At lower end: {}".format(np.min(distances) / (target_speed * update_time)))
    print("At higher end: {}".format(np.max(distances) / (target_speed * update_time)))

    # plt.scatter(x, y)
    # plt.show()

if __name__ == "__main__":

    main()
