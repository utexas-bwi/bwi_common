#!/usr/bin/env python

from nav_msgs.srv import GetMapResponse
from PIL import Image
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import os
import yaml

def loadMapFromFile(yaml_file):
    map_info = yaml.load(open(yaml_file, 'r'))
    resolution = map_info.get('resolution')
    origin = map_info.get('origin')
    negate = map_info.get('negate')
    occupied_thresh = map_info.get('occupied_thresh')
    free_thresh = map_info.get('free_thresh')

    image_file = map_info.get('image')
    if image_file[0] != '/': 
        yaml_file_dir = os.path.dirname(os.path.realpath(yaml_file))
        image_file = yaml_file_dir + '/' + image_file

    return loadMapFromImageFile(image_file, resolution,
          negate, occupied_thresh, free_thresh, origin)

def loadMapFromImageFile(image_file, resolution, negate, occupied_thresh,
                    free_thresh, origin):

    resp = GetMapResponse()

    image = Image.open(image_file)
    pix = image.load()

    image_size = image.size
    resp.map.info.width = image_size[0]
    resp.map.info.height = image_size[1]
    resp.map.info.resolution = resolution

    resp.map.info.origin.position.x = origin[0]
    resp.map.info.origin.position.y = origin[1]
    resp.map.info.origin.position.z = 0
    q = quaternion_from_euler(0,0,origin[2])
    resp.map.info.origin.orientation.x = q[0]
    resp.map.info.origin.orientation.y = q[1]
    resp.map.info.origin.orientation.z = q[2]
    resp.map.info.origin.orientation.w = q[3]

    test_pxl = pix[0,0]
    if isinstance(test_pxl, (list, tuple)):
      is_multi_layer = True
      num_layers = len(test_pxl)
    else:
      is_multi_layer = False
      num_layers = 1

    resp.map.data = [None] * image_size[0] * image_size[1]
    for j in range(image_size[1]):
      for i in range(image_size[0]):

        pxl = pix[i, j]

        if is_multi_layer:
          color_average = sum(pxl) / num_layers
        else:
          color_average = pxl

        if negate:
          occ = color_average / 255.0;
        else:
          occ = (255.0 - color_average) / 255.0;

        map_idx = resp.map.info.width * (resp.map.info.height - j - 1) + i 
        if (occ > occupied_thresh):
          resp.map.data[map_idx] = 100
        elif (occ < free_thresh):
          resp.map.data[map_idx] = 0
        else:
          resp.map.data[map_idx] = -1

    return resp

def saveMapToFile(map, yaml_file, image_file, negate, free_thresh,
                  occupied_thresh):

    map_info = {}

    map_info['resolution'] = map.info.resolution

    origin = [0.0, 0.0, 0.0]
    origin[0] = map.info.origin.position.x 
    origin[1] = map.info.origin.position.y 
    a = euler_from_quaternion([
        map.info.origin.orientation.x,
        map.info.origin.orientation.y,
        map.info.origin.orientation.z,
        map.info.origin.orientation.w
    ])
    origin[2] = a[2] #yaw
    map_info['origin'] = origin

    map_info['negate'] = 1 if negate else 0
    map_info['occupied_thresh'] = occupied_thresh
    map_info['free_thresh'] = free_thresh

    map_info['image'] = image_file
    if image_file[0] != '/': 
        yaml_file_dir = os.path.dirname(os.path.realpath(yaml_file))
        image_file = yaml_file_dir + '/' + image_file
    saveMapToImageFile(map, image_file, negate, free_thresh, occupied_thresh)

    with open(yaml_file, "w") as outfile:
        outfile.write(yaml.dump(map_info))
        outfile.close()

def saveMapToImageFile(map, image_file, negate, free_thresh, occupied_thresh):
    
    image = Image.new("L", (map.info.width, map.info.height))
    pixels = image.load()

    for j in range(map.info.height):
        for i in range(map.info.width):

            # The occupancy grid is vertically flipped
            map_idx = map.info.width * (map.info.height - j - 1) + i 

            # Setup negated image first
            if map.data[map_idx] == 100:
                pixels[i,j] = 255
            elif map.data[map_idx] == 0:
                pixels[i,j] = 0
            else:
                pixels[i,j] = 255 * ((free_thresh + occupied_thresh) / 2.0) 

            if not negate:
                pixels[i,j] = 255 - pixels[i,j]

    image.save(image_file)
