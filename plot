import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
import sys
import env


class Plotting:
    def __init__(self, x_start, x_goal, obs_rectangle):
        self.xI, self.xG = x_start, x_goal
        self.obs_rectangle = obs_rectangle

    def plot(self, path, name):
        fig, ax = plt.subplots()
        ax.set_facecolor('white')

        # Plot obstacles
        for (ox, oy, w, h) in self.obs_rectangle:
            ax.add_patch(patches.Rectangle((ox, oy), w, h, edgecolor='black', facecolor='gray', fill=True))

        # Plot start and goal
        plt.plot(self.xI[0], self.xI[1], "bs", linewidth=3, label="Start")
        plt.plot(self.xG[0], self.xG[1], "gs", linewidth=3, label="Goal")

        # Plot path
        if path:
            plt.plot([x[0] for x in path], [x[1] for x in path], "r", linewidth=2, label="Path")

        plt.title(name)
        plt.axis("equal")
        plt.show()
