import matplotlib.pyplot as plt
import numpy as np

import pandas as pd
import glob
import ipdb
import re

def plot(fname):

    fig = plt.figure()
    ax = fig.gca()

    df = pd.read_csv(fname, index_col=0)
    X = df.columns.values
    Y = df.index.values

    XX, YY = np.meshgrid(X, Y)
    ZZ = np.empty( XX.shape )

    for i in range(XX.shape[0]):
        for j in range(XX.shape[1]):

            x = XX[i,j]
            y = YY[i,j]

            ZZ[i,j] = df[x].ix[y]

    X  = X.astype(np.float)
    Y  = Y.astype(np.float)
    XX = XX.astype(np.float)
    YY = YY.astype(np.float)

    plt.pcolor(XX, YY, ZZ)
    ax.set_xlim([X[0], X[-1]])
    ax.set_ylim([Y[0], Y[-1]])
    ax.set_xlabel('longitude')
    ax.set_ylabel('latitude')
    ax.set_title(fname.replace(".txt", ""))

    plt.colorbar()

    fname = fname.replace('csv', 'png')
    plt.savefig(fname)

    plt.clf()

def main():
    flist = ['elevation.csv', 'terrain.csv']

    for i,f in enumerate(flist):
        plot(f)

if __name__ == '__main__':
    main()
