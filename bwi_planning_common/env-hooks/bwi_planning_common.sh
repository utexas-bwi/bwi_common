#!/usr/bin/env sh

annotate_map() {
    map_name=$(basename $PWD)
    rosrun bwi_planning_common logical_marker _data_directory:=$PWD _map_file:=$map_name.yaml
}