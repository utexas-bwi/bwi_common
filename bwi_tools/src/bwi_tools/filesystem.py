#!/usr/bin/env python

import os
import fnmatch
import glob

def expand_path_to_filelist(path):
    recursive_path_split = path.split('/**/')
    if len(recursive_path_split) == 2:
        files = [os.path.join(dirpath, f)
                for dirpath, dirnames, files in os.walk(recursive_path_split[0])
                for f in fnmatch.filter(files, recursive_path_split[1])]
    else:
        files = glob.glob(path)
    
    for file in files:
        if not os.path.isfile(file):
            raise ValueError('path %s expanded to include %s, which is not a file' % (path, file))

    return files
