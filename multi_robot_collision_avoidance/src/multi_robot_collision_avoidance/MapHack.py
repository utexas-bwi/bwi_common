import os

import yaml
from PIL import Image
import numpy as np

# Ros related packages
import rospkg

class MapHack:
    """MapHack()

    Create a new Maphack object.
    It automatically reads map, meta and segmentation data from utexas_gdc ros package.

    This object will extract 4 distance features of input point from map.
    Use visualization functions to see how it works.

    d1 = distance from chicken robot to point
    d2 = distance from bold robot to point
    d3 = distance from left wall to point
    d4 = distance from right wall to point
    ================================
    Input
    simulation: True if running on simulation, False otherwise.
    """
    def __init__(self, simulation):
        mapDir =  {True: "maps/simulation/3ne/3ne.pgm",
                   False:"maps/real/3/3.pgm"}
        metaDir = {True: "maps/simulation/3ne/3ne.yaml",
                   False:"maps/real/3/3.yaml"}
        segDir =  {True: "maps/simulation/3ne/locations.pgm",
                   False:"maps/real/3/locations.pgm"}
        # Read nessesary information from utexas_gdc package.
        rospack = rospkg.RosPack()
        pkgPath = rospack.get_path('utexas_gdc')
        mapFile = os.path.join(pkgPath, mapDir[simulation])
        segFile = os.path.join(pkgPath, segDir[simulation])
        metaFile = os.path.join(pkgPath, metaDir[simulation])

        with open(metaFile, 'r') as file:
            self.metaData = yaml.safe_load(file)
        self.mapData = np.array(   Image.open(mapFile) )
        self.h, self.w = self.mapData.shape
        self.segData = np.asarray( Image.open(segFile) )

        self.filter = self.generate_filter([6,18,21,26,29]) # filter for simulation env

    def pose2pix(self, pose):
        """Convert real pose (x,y) [m] -> pixel pose [y,x]
        ================================
        Input
        pose: (x,y) [m]
        ================================
        Return
        [y,x] [pixel]
        """
        coord = np.array(pose)
        pix = (coord - self.metaData['origin'][:-1]) / self.metaData['resolution']
        pix = [self.h, 0] + pix[::-1] * [-1,1]
        return pix
    def pix2pose(self, pix):
        """Convert pixel pose [y,x] -> real pose (x,y) [m]
        ================================
        Input
        pose: [y,x] [pixel]
        ================================
        Return
        (x,y) [m]
        """
        coord = ([self.h, 0] + np.array(pix) * [-1, 1])[::-1] # now [x,y]
        pose = coord * self.metaData['resolution'] + self.metaData['origin'][:-1]

        return pose
    def sampling(self, sample_centre, window, n_sample):
        """This function sample uniform distribution on a given rectangular area.
        ================================
        INPUT
        sample_centre: origin of sampling area (y,x) [pixel]
        window: size of sampling window [lower_left,upper_right], 2x2 # change to single window after integrate chicken to siren.
        n_sample: number of sampling
        ================================
        Return
        Samples (y's,x's)"""
        bound = np.stack([sample_centre, sample_centre])
        bound = bound + window
        bound = np.clip(bound, [0,0], [self.h, self.w])
        samples = np.random.uniform(bound[0], bound[1], (n_sample,2))

        return samples

    def generate_filter(self, index):
        """Generate filter that exclude all the non-hallway sample points
        by comparing it with segmentation image
        =============================
        Input
        index : the index of hallway; color [numpy array]"""
        def sample_filter(samples):
            qs = samples.astype(int)
            idx = np.isin(self.segData[qs[:,0], qs[:,1]], index)
            return samples[idx]
        return sample_filter

    def _dists(self, v1, v2):
        v1 = np.array(v1).reshape(-1,2)
        v2 = np.array(v2).reshape(-1,2)
        return np.linalg.norm(v1-v2, axis=1)

    def extractFeature(self, coward, bold, wps, search_window = 1.6):
        """Extract features from given waypoint position.
        d1 := d( coward robot -> waypoint )
        d2 := d( bold robot -> waypoint )
        d3 := d( waypoint -> closest wall pt)
        d4 := d( waypoint -> far wall pt)
        ===============
        INPUT
        coward : pixelwise position of coward robot
        bold   : pixelwise position of bold   robot
        wp     : pixelwise position of waypoints
        search_window : size of wall searching window
        ===============
        OUTPUT
        features of all waypoints. [# waypoints, 4]"""
        window = search_window/self.metaData['resolution']
        features = np.empty((len(wps), 4))

        features[:,0] = self._dists(coward, wps)
        features[:,1] = self._dists(bold, wps)

        for i, wp in enumerate(wps):
            # Define quantized search area. area := sampling area(=0) + window
            lower_bound = np.trunc(wp - window)
            upper_bound = np.ceil (wp + window)
            # clip search area within the map image.
            lower_bound = np.clip(lower_bound, [0, 0], [self.h, self.w]).astype(int)
            upper_bound = np.clip(upper_bound, [0, 0], [self.h, self.w]).astype(int)
            # Find wall pts in the search region
            # disp := displacement
            disp = np.argwhere(self.mapData[lower_bound[0]:upper_bound[0],
                                            lower_bound[1]:upper_bound[1]] ==0 )
            disp = (disp + lower_bound) - wp
            dist = self._dists(disp, [0,0])
            if(len(dist) == 0):
                features[i,2] = features[i,3] = window
                continue
            first_wall_pt_idx = np.argmin(dist)
            features[i,2] = dist[first_wall_pt_idx]

            direction = (np.cross(disp[first_wall_pt_idx], bold-coward) > 0)

            Rwps = wp - lower_bound
            closest_pt = disp[first_wall_pt_idx] + Rwps

            ind = (np.dot(disp, disp[first_wall_pt_idx])<-0.)

            disp = disp[ind,:]
            dist = dist[ind]

            if(np.sum(ind) == 0):
                features[i,3] = 32
            else:
                idx = np.argmin(dist)
                features[i,3] = dist[idx]

            if(direction == True):
                tmp = features[i,2]
                features[i,2] = features[i,3]
                features[i,3] = tmp
        return features * self.metaData["resolution"]
