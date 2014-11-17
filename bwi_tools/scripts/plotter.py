#!/usr/bin/env python

import argparse
import bwi_tools.graph as graph
import bwi_tools.filesystem as fs
import json
import matplotlib.pyplot as plt; plt.rcdefaults()

parser = argparse.ArgumentParser()
parser.add_argument("data", help="The directory containing csv files or a single csv file.", type=str)
parser.add_argument("output", help="Column name in csv to plot.", type=str)
parser.add_argument("--plot-type", help="Plot type. One of ['line', 'bar', '3d'].", type=str)
parser.add_argument("--filter", help="Comma separated primary grouping filter. Defaults to 'name' if available",
                    type=str)
parser.add_argument("--secondary-filter", help="Comma separated secondary grouping filter.", type=str)
parser.add_argument("--name-mappings", help="JSON string with mapping from column name to printed name.", type=str)
parser.add_argument("--attempt_auto_mapping", help="Attempt to perform automatic cleanup of CSV column name.", 
                    action='store_true')

args = parser.parse_args()

if not args.plot_type:
    args.plot_type = 'bar'

name_mappings = None
if args.name_mappings is not None:
    name_mappings = json.loads(args.name_mappings)

fig, ax, rects, means= \
        graph.draw_from_data_frame(fs.expand_path_to_filelist(args.data), 
                                   args.output, args.plot_type, args.filter, args.secondary_filter, 
                                   args.attempt_auto_mapping, name_mappings)


fig = plt.gcf()
fig.set_size_inches(5,4)
pad_inches = 0.1
if args.plot_type == "3d":
    azim = raw_input("Enter Azimuth [Hit enter to default to -70]: ")
    if azim is None or azim == "":
        azim = -70
    elev = raw_input("Enter Elevation [Hit enter to default to 25]: ")
    if elev is None or elev == "":
        elev = 25
    ax.view_init(elev=float(elev), azim=float(azim))

plt.savefig('out.png',bbox_inches='tight',pad_inches=pad_inches,dpi=100)
plt.show()
