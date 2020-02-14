import os

import numpy as np
import matplotlib.pyplot as plt
from skimage import feature

class MapHack:
    """MapHack(dir of map, dir of meta data(.yaml))

    Create a new Maphack object with given map file and meta data file.
    This object will extract 4 distance features of input point from map.
    Use visualization functions to see how it works.

    d1 = distance from chicken robot to point
    d2 = distance from bold robot to point
    d3 = distance from left wall to point
    d4 = distance from right wall to point
    """
    def __init__(self, map_path, metadata_path):
        self.map_path = map_path
        self.metadata_path = metadata_path
        self.mapImage = self.__loadMapData(map_path)
        self.meta = self.__loadMetaData(metadata_path)
        self.mapEdge = self.__buildEdgeMap(self.mapImage)

    def __loadMapData(self, map_path):
        mapImage = plt.imread(map_path)
        self.h_, self.w_ = mapImage.shape
        return mapImage

    def __loadMetaData(self, metadata_path):
        meta = {}
        with open(metadata_path) as f:
            for line in f:
                key, val = line.strip().split(": ")
                try:
                    if(val[0]=="["): # if data is array
                        meta[key] = np.fromstring(val[1:-1], dtype=float, sep=", ")
                    else:
                        meta[key] = float(val)
                except:
                    meta[key] = val
        if(meta['image'] != self.map_path.split("/")[-1]):
            print("meta data does not match to map data.")
            print("{} != {}".format(meta['image'], self.map_path.split("/")[-1]))
        return meta

    def __buildEdgeMap(self, image, sigma = 0.1):
        # To measure the distance to the wall, the interior points are not needed.
        # Edge map will reduce the search space while maintaining purformance
        mapEdge = feature.canny(self.mapImage, sigma=sigma)

        strictEdge = mapEdge * ~self.mapImage
        return strictEdge

    def showMap(self, mode='original'):
        plt.figure(figsize=(32,12))
        if(mode == 'original'):
            plt.imshow(self.mapImage, cmap='gray')
        elif(mode == 'edge'):
            plt.imshow(self.mapEdge, cmap='gray')
        else:
            print("Available mode: [original, edge]")
        return

    def showMetaData(self):
        print("Meta data of current map:")
        for key in self.meta:
            print("{}: {}".format(key, self.meta[key]))
        return
    def __dist(self, v1, v2=np.zeros(2)):
        if(v1.size==2):
            return np.linalg.norm(v1-v2)
        return np.linalg.norm(v1-v2, axis=1)

    # Change pixel position to meter position
    def pos2pixConverter(self, pos):
        pix = (pos - self.meta['origin'][:-1])/self.meta['resolution']
        if(pix.size==2):
            pix[1] = self.h_ - pix[1]
        else:
            pix[:,1] = self.h_ - pix[:,1]
        return pix.astype(int)

    def pix2posConverter(self, pix):
        if(pix.size==2):
            pix[1] =  self.h_ - pix[1]
        else:
            pix[:,1] = self.h_ - pix[:,1]
        pos = pix * self.meta['resolution'] + self.meta['origin'][:-1]
        return pos

    def extractFeatures(self, chickenRobotPos, boldRobotPos, waypointPos,
                        search_window = 1.5, mode = 'pos', verbose=False):
        extractedFeatures = np.zeros(4) - 1 # For debuging purpose. -1 = error.
        wall_disp = np.zeros((2,2)) # displacement of the wall from waypoint coordinate

        window_pix = search_window / self.meta['resolution']
        if(mode == 'pix'):
            px, py = waypointPos
            x, y = self.pix2posConverter(waypointPos)
        elif(mode == 'pos'):
            x,y = waypointPos
            px, py = self.pos2pixConverter(waypointPos)
        else:
            print("mode = ['pix', 'pos']")

        bold_pix = self.pos2pixConverter(boldRobotPos)
        chicken_pix = self.pos2pixConverter(chickenRobotPos)
        extractedFeatures[:2] = self.__dist([chicken_pix, bold_pix]-np.array([px,py]))

        # clip search region
        lowerY = int(max(py-window_pix, 0))
        upperY = int(min(py+window_pix+1, self.mapEdge.shape[0]))
        lowerX = int(max(px-window_pix, 0))
        upperX = int(min(px+window_pix+1, self.mapEdge.shape[1]))
        search_area = self.mapEdge[lowerY:upperY, lowerX:upperX]
        # convert index to displacement
        delta = np.array([min(py-lowerY,window_pix), min(px-lowerX,window_pix)])
        displacement = np.argwhere(search_area!=0) - delta   # [y, x]
        distance = self.__dist(displacement)

        idx = np.argsort(distance)
        if(idx.size == 0):
            if(verbose): print("({},{}) is an open area. PIXEL LOCATION: ({},{})".format(x,y, px, py))
            wall_disp[0] = wall_disp[1] = [window_pix,0]
        else:
            displacement = displacement[idx]
            wall_disp[0] = displacement[0]

            moveDirection = (np.array(bold_pix) - np.array(chicken_pix))[::-1]
            baseline = np.cross(moveDirection, wall_disp[0])
            indicator = np.cross(moveDirection, displacement) * baseline
            mask = (indicator<0)
            if(np.sum(mask)==0):
                if(verbose): print("({},{}) is an semi-open area. PIXEL LOCATION: ({},{})".format(x,y, px, py))
                wall_disp[1] = [window_pix,0]
            else:
                displacement = displacement[mask]
                wall_disp[1] = displacement[0]
            if(baseline<0):
                wall_disp = wall_disp[::-1]

        # DEBUG
        if(verbose):
            wall_idx = np.argwhere(search_area!=0)
            plt.imshow(search_area, cmap='gray') # show search area
            plt.scatter(px-lowerX,py-lowerY) # show waypoint
            plt.scatter(wall_idx.T[1], wall_idx.T[0]) # Show wall points
            plt.scatter(displacement[:,1]+delta[1], displacement[:,0]+delta[0])
            plt.scatter(wall_disp[0][1]+delta[1], wall_disp[0][0]+delta[0], color='r') # Show closest wall
            plt.scatter(wall_disp[1][1]+delta[1], wall_disp[1][0]+delta[0], color='r')
            plt.savefig("HowModuleWorks.png")

        extractedFeatures[2:] = self.__dist(wall_disp)
        return extractedFeatures * self.meta['resolution']
