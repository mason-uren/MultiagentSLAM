# Created by Mason U'Ren on 10/25/2018

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import random
from enum import Enum

class Distinctions(Enum):
    DATA = 0
    NOISE = 1
    DISPLAY = 2


class Grapher:

    def __init__(self, sampling_rate, freq):
        self.plots = []
        self.sampling_rate = sampling_rate
        self.freq = freq

    def generatePoints(self, with_noise=False, x_shift=0, y_shift=0, display=True):
        x = np.arange(self.sampling_rate)
        y = (
            [(np.sin((2 * np.pi * self.freq * (i / self.sampling_rate)) + x_shift) + y_shift) for i in x]
            , [(np.sin((2 * np.pi * self.freq * ((i + random.uniform(-5, 5)) / self.sampling_rate)) + x_shift) + y_shift) for i in
               x])[with_noise]
        sin_wave = pd.DataFrame({
            'x' : x,
            'y' : y,
        })

        self.plots.append([sin_wave, with_noise, display])

    def plot_sin_wave(self):
        for axes, index in zip(self.plots, range(self.plots.__len__())):
            if axes[Distinctions.DISPLAY.value]:
                plt.plot(axes[Distinctions.DATA.value]['x'], axes[Distinctions.DATA.value]['y'], label='{}'.format(index))

    def getCentroids(self):
        centroids = []
        x_df = []
        y_df = []
        for axes in self.plots:
            if axes[Distinctions.NOISE.value]:
                x_df.append(axes[Distinctions.DATA.value]['x'])
                y_df.append(axes[Distinctions.DATA.value]['y'])
        x_df = pd.DataFrame(x_df)
        y_df = pd.DataFrame(y_df)
        for row in range(x_df.shape[1]):
            centroids.append([
                x_df[row].sum() / 3, # TODO: because there are three rovers
                y_df[row].sum() / 3
            ])
        return pd.DataFrame(centroids)

    def displayCentriods(self):
        centroids = self.getCentroids()
        plt.plot(centroids[0], centroids[1])

    @staticmethod
    def showGraphs():
        plt.legend()
        plt.show()

if __name__ == '__main__':
    sin_wave = Grapher(100, 2)
    sin_wave.generatePoints()
    sin_wave.generatePoints(with_noise=True, display=False)
    sin_wave.generatePoints(with_noise=True, y_shift=4, display=False)
    sin_wave.generatePoints(with_noise=True, x_shift=np.pi, y_shift=8)

    sin_wave.plot_sin_wave()
    sin_wave.displayCentriods()


    Grapher.showGraphs()
