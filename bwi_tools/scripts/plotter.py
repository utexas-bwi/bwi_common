#!/usr/bin/env python

import argparse
import bwi_tools.graph as graph
import matplotlib.pyplot as plt; plt.rcdefaults()

parser = argparse.ArgumentParser()
parser.add_argument("data", help="The directory containing csv files or a single csv file.", type=str)
parser.add_argument("output", help="Column name in csv to plot.", type=str)
parser.add_argument("--plot-type", help="Plot type. One of ['line', 'bar', '3d'].", type=str)
args = parser.parse_args()

if not args.plot_type:
    args.plot_type = 'bar'

fig, ax, rects, means= \
        graph.draw_from_data_frame(args.data, args.output, args.plot_type)

fig = plt.gcf()
fig.set_size_inches(6,4)
plt.savefig('out.png',bbox_inches='tight',pad_inches=0.1,dpi=100)

plt.show()

