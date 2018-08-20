#!/usr/bin/env sh

annotate_map() {
    map_name=$(basename $PWD)
    yaml_file=$map_name.yaml
    if [ ! -f "$yaml_file" ]; then
        yaml_file=map.yaml
    fi
    rosrun bwi_planning_common logical_marker _data_directory:=$PWD _map_file:=$yaml_file
}