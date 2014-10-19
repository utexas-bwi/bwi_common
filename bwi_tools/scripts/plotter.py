#!/usr/bin/env python

import argparse
import bwi_tools.graph as graph
import bwi_tools.filesystem as fs
import matplotlib.pyplot as plt; plt.rcdefaults()

parser = argparse.ArgumentParser()
parser.add_argument("data", help="The directory containing csv files or a single csv file.", type=str)
parser.add_argument("output", help="Column name in csv to plot.", type=str)
parser.add_argument("--plot-type", help="Plot type. One of ['line', 'bar', '3d'].", type=str)
parser.add_argument("--filter", help="Comma separated primary grouping filter. Defaults to 'name' if available",
                    type=str)
parser.add_argument("--secondary-filter", help="Comma separated secondary grouping filter.", type=str)
parser.add_argument("--name-mapping-file", help="JSON file with mapping from column name to printed name.", type=str)
parser.add_argument("--attempt_auto_mapping", help="Attempt to perform automatic cleanup of CSV column name.", 
                    action='store_true')

args = parser.parse_args()

if not args.plot_type:
    args.plot_type = 'bar'

fig, ax, rects, means= \
        graph.draw_from_data_frame(fs.expand_path_to_filelist(args.data), 
                                   args.output, args.plot_type, args.filter, args.secondary_filter, 
                                   args.attempt_auto_mapping, args.name_mapping_file)

fig = plt.gcf()
fig.set_size_inches(6,4)
plt.savefig('out.png',bbox_inches='tight',pad_inches=0.1,dpi=100)

plt.show()
