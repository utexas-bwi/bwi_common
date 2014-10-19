#!/usr/bin/env python

import collections
import copy
import numpy as np
import itertools
import math
import matplotlib.cm as cm
import matplotlib.colors as colors
import matplotlib.pyplot as plt; plt.rcdefaults()
from mpl_toolkits.mplot3d import Axes3D
import re
import scipy.stats as stats
import pandas as pd

# Keep the following at different length to produce more distinct combinations
METHOD_COLORS = ['yellow', 'red', 'aqua', 'green', 'lightgray', 'blue']
METHOD_HATCH = ['/', '\\', 'x', '*', 'o', 'O', '.']
LINE_COLORS = ['red', 'blue', 'green']
LINE_HATCH = [(20,0),(20,5),(5,5),(15,5,5,5),(15,5,2,5)]

def mean_standard_error(data):
    a = 1.0 * np.array(data)
    n = len(a)
    m, se = np.mean(a), stats.sem(a)
    return m, se

def mean_confidence_interval(data, confidence=0.95):
    a = 1.0 * np.array(data)
    n = len(a)
    m, se = np.mean(a), stats.sem(a)
    h = se * stats.t._ppf((1+confidence)/2., n-1)
    return m, h

def is_significant(a, b, confidence=0.95):
    t, p = stats.ttest_ind(a, b)
    return p < (1.0 - confidence)

def mix_colors(c1, c2, amount):
    return [int((1 - amount) * c1[i] + amount * c2[i]) for i in range(3)]

def color_to_html_string(c):
    # http://stackoverflow.com/questions/13998901/generating-a-random-hex-color-in-python
    return '#%02x%02x%02x' % (c[0],c[1],c[2])

def draw_bar_chart(samples, top_level_names, second_level_names=None, 
                   title=None, xlabel=None, ylabel=None, color=None,
                   bottom=None, yticklabels=None):

    # So, samples can either contain a list of lists. The top level list
    # contains top level groups, and the second level list contains actual
    # samples (top_level_grouping_only = true)

    # Alternatively, samples may be a list of lists of lists, with top-level 
    # groups, second level groups and actual samples. (top_level_grouping_only)

    means = []
    confs = []
    second_level_grouping_available = \
            isinstance(samples[0][0], collections.Sequence)
    top_level_methods = len(samples)

    if second_level_grouping_available: 
        second_level_methods = len(samples[0])
        samples2 = samples
    else:
        # Create artificial second level grouping
        second_level_methods = 1
        samples2 = [[samples[i]] for i in range(top_level_methods)]

    for i in range(top_level_methods):
        means.append([])
        confs.append([])

    for i in range(top_level_methods):
        for j in range(second_level_methods):
            m, h = mean_standard_error(samples2[i][j])
            means[i].append(m)
            confs[i].append(h)

    ind = np.arange(second_level_methods)
    width = 1.0 / (top_level_methods + 1)
    fig, ax = plt.subplots()
    rects = []
    for i in range(top_level_methods):
        barhatch = None
        barcolor = color
        if color is None:
            barcolor = METHOD_COLORS[i % len(METHOD_COLORS)]
            barhatch = METHOD_HATCH[i % len(METHOD_HATCH)]
        rect = ax.bar(ind + i*width, means[i], width,
                      color=barcolor, 
                      hatch=barhatch,
                      yerr=confs[i], ecolor='black', bottom=bottom)
        rects.append(rect)

    if xlabel:
        ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)
    if title:
        ax.set_title(title)

    if second_level_grouping_available:
        ax.set_xticks(ind+0.5-width/2)
        if second_level_names:
            ax.set_xticklabels(second_level_names)

    if yticklabels:
        ax.set_yticklabels(yticklabels)

    if top_level_names:
        ax.legend(rects, top_level_names)

    return fig, ax, rects, means

def draw_line_graph(samples, top_level_names, second_level_names=None, 
                    title=None, xlabel=None, ylabel=None, yticklabels=None):

    # So, samples can either contain a list of lists. The top level list
    # contains top level groups, and the second level list contains actual
    # samples (top_level_grouping_only = true)

    # Alternatively, samples may be a list of lists of lists, with top-level 
    # groups, second level groups and actual samples. (top_level_grouping_only)

    means = []
    confs = []
    second_level_grouping_available = \
            isinstance(samples[0][0], collections.Sequence)
    top_level_methods = len(samples)

    if second_level_grouping_available: 
        second_level_methods = len(samples[0])
        samples2 = samples
    else:
        # Create artificial second level grouping
        second_level_methods = 1
        samples2 = [[samples[i]] for i in range(top_level_methods)]

    for i in range(top_level_methods):
        means.append([])
        confs.append([])

    for i in range(top_level_methods):
        for j in range(second_level_methods):
            m, h = mean_standard_error(samples[i][j])
            means[i].append(m)
            confs[i].append(h)

    ind = np.arange(second_level_methods)
    fig, ax = plt.subplots()
    rects = []
    for i in range(top_level_methods):
        rect, = ax.plot(np.arange(0, second_level_methods), means[i],
                        color=LINE_COLORS[i%len(LINE_COLORS)],
                        dashes=LINE_HATCH[i%len(LINE_HATCH)],
                        linewidth = 4)
        rects.append(rect)

    if xlabel:
        ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)
    if title:
        ax.set_title(title)

    if second_level_grouping_available:
        tick_multiplier = int(math.ceil(float(second_level_methods)/float(len(second_level_names))))
        ax.set_xticks(tick_multiplier * ind)
        if second_level_names:
            ax.set_xticklabels(second_level_names)

    if yticklabels:
        ax.set_yticklabels(yticklabels)
    ax.legend(rects, top_level_names, handlelength=4) #mode='expand', ncol=4)

    return fig, ax, rects, means

def draw_3d_bar_chart(samples, top_level_names=None, second_level_names=None, 
                      title=None, xlabel=None, ylabel=None, zlabel=None,
                      xtickrotation=0, flip_y=True, third_level_names=None):

    # So, samples can either contain a list of lists. The top level list
    # contains top level groups, and the second level list contains actual
    # samples (top_level_grouping_only = true)

    # Alternatively, samples may be a list of lists of lists, with top-level 
    # groups, second level groups and actual samples. (top_level_grouping_only)

    means = []
    confs = []
    second_level_grouping_available = \
            isinstance(samples[0][0], collections.Sequence)
    top_level_methods = len(samples)

    if second_level_grouping_available: 
        second_level_methods = len(samples[0])
        samples2 = samples
    else:
        # Create artificial second level grouping
        second_level_methods = 1
        samples2 = [[samples[i]] for i in range(top_level_methods)]

    for i in range(top_level_methods):
        means.append([])
        confs.append([])

    for i in range(top_level_methods):
        for j in range(second_level_methods):
            m, h = mean_standard_error(samples[i][j])
            means[i].append(m)
            confs[i].append(h)

    ind = np.arange(second_level_methods)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    rects = []
    xpos = np.array([])
    ypos = np.array([])
    zpos = np.array([])
    dx = np.array([])
    dy = np.array([])
    dz = np.array([])
    for j in range(second_level_methods):
        for i in range(top_level_methods):
            xpos = np.append(xpos, i)
            if flip_y:
                ypos = np.append(ypos, second_level_methods - j - 1)
            else:
                ypos = np.append(ypos, j)
            zpos = np.append(zpos, 0)
            dx = np.append(dx, 1.0)
            dy = np.append(dy, 0.5)
            dz = np.append(dz, means[i][j])

    #http://stackoverflow.com/questions/11950375/apply-color-map-to-mpl-toolkits-mplot3d-axes3d-bar3d
    offset = dz + np.abs(dz.min())
    fracs = offset.astype(float)/offset.max()
    norm = colors.normalize(fracs.min(), fracs.max())
    colors_t = cm.jet(norm(fracs) / 2 + 0.5)

    # for xs, ys, zs, dxs, dys, dzs, colors_ts in zip(xpos, ypos, zpos, dx, dy, dz, colors_t):
    #     rects.append(ax.bar3d(xs, ys, zs, dxs, dys, dzs, color=colors_ts, zsort=''))
    rects = ax.bar3d(xpos, ypos, zpos, dx, dy, dz, color=colors_t, zsort=True)

    if xlabel:
        ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)
    if zlabel:
        ax.set_zlabel(zlabel)
    if title:
        ax.set_title(title)

    if second_level_grouping_available:
        ax.set_yticks(ind + 0.5)
        if second_level_names:
            if flip_y:
                second_level_names.reverse()
            ax.set_yticklabels(second_level_names)

    if third_level_names:
        ax.set_zticklabels(third_level_names)

    tick_multiplier = int(math.ceil(float(top_level_methods)/float(len(top_level_names))))
    ax.set_xticks(tick_multiplier * np.arange(len(top_level_names)) + 0.5)
    if top_level_names:
        ax.set_xticklabels(top_level_names, rotation=xtickrotation)
#    ax.legend(rects, top_level_names, mode='expand', ncol=3)

    return fig, ax, rects, means

def is_greek_alphabet(str):
    return str.lower() in ['alpha', 'beta', 'gamma', 'delta', 'epsilon', 'zeta', 'eta', 'theta', 'iota', 'kappa',
                           'lambda', 'mu', 'nu', 'xi', 'omicron', 'pi', 'rho', 'sigma', 'tau', 'upsilon', 'phi',
                           'chi', 'psi', 'omega']

def format_word(str):
    if is_greek_alphabet(str):
        return '$\\' + str + '$'
    else:
        return str.title()

#TODO fix the name mapping file stuff
def get_formatted_name(name, name_mapping_file=None):
    name = re.sub('(.)([A-Z][a-z]+)', r'\1 \2', name) # Convert camelCase to space separated words
    name = name.replace('_', ' ') # Convert snake_case to space separate words
    name = ''.join([format_word(x) for x in name.split(' ')]) # join words while converting greek letters.
    return name

def get_formatted_combination_name(name_dict):
    formatted_name = ""
    for key, value in name_dict.iteritems():
        if key == "name":
            continue
        formatted_name += get_formatted_name(key) + "=" + str(value) + ","
    formatted_name = formatted_name[:-1]

    if "name" in name_dict:
        if formatted_name == "":
            formatted_name = str(name_dict["name"])
        else:
            formatted_name = str(name_dict["name"]) + "[" + formatted_name + "]"
    return formatted_name

def draw_from_data_frame(filename, output, plot_type, filter=None, secondary_filter=None, 
                         attempt_auto_mapping=True, name_mapping_file=None):

    data_per_file = []
    for file in filename:
        data_per_file.append(pd.read_csv(file))
    data = pd.tools.merge.concat(data_per_file)

    # Check if the output column exists in the data frame.
    if output not in data:
        print "Output column name not in data!"
        return

    # Get primary and secondary filters.
    if (filter is None) and "name" in data:
        filter = "name"

    primary_filters = None
    if filter is not None:
        primary_filters = filter.split(",")

    secondary_filters = None
    if secondary_filter is not None:
        secondary_filters = secondary_filter.split(",")

    title = None
    xlabel = None
    ylabel = None
    zlabel = get_formatted_name(output)
    samples = []
    top_level_names = None
    second_level_names = None
    is_first_primary_combination = True

    if primary_filters is None:
        samples.append(data[output].tolist())
    else:
        if len(primary_filters) == 1 and filter != "name":
            xlabel = get_formatted_name(filter)
        else:
            xlabel = 'Methods'
        primary_unique_values = []
        primary_filters_copy = copy.deepcopy(primary_filters)
        for i, pf in enumerate(primary_filters_copy):
            if len(data[pf].unique().tolist()) == 1: # This column is not really being used. 
                primary_filters.remove(pf)
            else:
                primary_unique_values.append(data[pf].unique().tolist())
        
        all_possible_primary_combinations = list(itertools.product(*primary_unique_values))
        
        top_level_names = []
        for primary_combination in all_possible_primary_combinations:
            combination_data = data
            combination_name = {}
            for i, filter_value in enumerate(primary_combination):
                combination_name[primary_filters[i]] = filter_value
                combination_data = combination_data[combination_data[primary_filters[i]] == filter_value]
            combination_name = get_formatted_combination_name(combination_name)

            # Now that we have combination data, apply secondary filtering if necessary
            if secondary_filters is None:
                combination_samples = combination_data[output].tolist()
            else:
                secondary_unique_values = []
                secondary_filters_copy = copy.deepcopy(secondary_filters)
                for i, pf in enumerate(secondary_filters_copy):
                    secondary_unique_values.append(combination_data[pf].unique().tolist())
                
                all_possible_secondary_combinations = list(itertools.product(*secondary_unique_values))

                combination_samples = []
                if is_first_primary_combination:
                    second_level_names = []
                    if len(secondary_filters) == 1 and filter != "name":
                        ylabel = get_formatted_name(secondary_filter)
                    else:
                        ylabel = 'Methods'
                for secondary_combination in all_possible_secondary_combinations:
                    secondary_combination_data = data
                    secondary_combination_name = {}
                    for i, filter_value in enumerate(secondary_combination):
                        secondary_combination_name[secondary_filters[i]] = filter_value
                        secondary_combination_data = secondary_combination_data[secondary_combination_data[secondary_filters[i]] == filter_value]
                    secondary_combination_name = get_formatted_combination_name(secondary_combination_name)
                    secondary_combination_samples = combination_data[output].tolist()

                    if is_first_primary_combination:
                        second_level_names.append(secondary_combination_name)
                    combination_samples.append(secondary_combination_samples)

            top_level_names.append(combination_name)
            samples.append(combination_samples)
            is_first_primary_combination = False

    if plot_type == 'line':
        ylabel = zlabel
        return draw_line_graph(samples. top_level_names, second_level_names, title, xlabel, ylabel)
    elif plot_type == '3d':
        return draw_3d_bar_chart(samples, top_level_names, second_level_names, title, xlabel, ylabel, zlabel)
    else:
        ylabel = zlabel
        return draw_bar_chart(samples, top_level_names, second_level_names, title, xlabel, ylabel)
