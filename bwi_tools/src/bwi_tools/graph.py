#!/usr/bin/env python

import collections
import numpy as np
import matplotlib.pyplot as plt; plt.rcdefaults()
import scipy.stats as stats

# Keep the following at different length to produce more distinct combinations
METHOD_COLORS = ['yellow', 'red', 'aqua', 'green', 'lightgray', 'blue']
METHOD_HATCH = ['/', '\\', 'x', '*', 'o', 'O', '.']

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

def draw_bar_chart(samples, top_level_names, second_level_names=None, 
                  title=None, xlabel=None, ylabel=None):

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
        rect = ax.bar(ind + i*width, means[i], width,
                      color=METHOD_COLORS[i % len(METHOD_COLORS)], 
                      hatch=METHOD_HATCH[i % len(METHOD_HATCH)],
                      yerr=confs[i])
        rects.append(rect)

    if xlabel:
        ax.set_ylabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)
    if title:
        ax.set_title(title)

    if second_level_grouping_available:
        ax.set_xticks(ind+0.5-width/2)
        if second_level_names:
            ax.set_xticklabels(second_level_names)

    ax.legend(rects, top_level_names)

    return fig, ax, rects, means

