#!/usr/bin/env python

import collections
import numpy as np
import math
import matplotlib.cm as cm
import matplotlib.colors as colors
import matplotlib.pyplot as plt; plt.rcdefaults()
from mpl_toolkits.mplot3d import Axes3D
import scipy.stats as stats

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
            m, h = mean_standard_error(samples[i][j])
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
        ax.set_xticks(ind)
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

