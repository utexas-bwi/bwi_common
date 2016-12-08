# BWI Person Detection

Person detection using pcl\_people module. Modified from [this example](http://pointclouds.org/documentation/tutorials/ground\_based\_rgbd\_people\_detection.php). Original repo at [here](https://github.com/Marcus-Zhu/bwi_pcl_detection).

## Launch Files

+ calibrate\_v2.launch: calibration launch file for segbot V2. Launch before person detection. Calibration file will be generated containing ground coefficients.  
+ person\_detection\_v2.launch: run person detection node and record person position.  
+ person\_detection\_v2\_norecord.launch: run person detection node without logging.


+ calibrate\_v3.launch: calibration launch file for segbot V3. Launch before person detection. Calibration file will be generated containing ground coefficients.  
+ person\_detection\_v3.launch: run person detection node and record person position.  
+ person\_detection\_v3\_norecord.launch: run person detection node without logging.

## Subscribed Topics

nav\_kinect/depth\_registered/points

## Published Topics

+ pcl\_detector/marker: visualization\_msgs/Marker
+ pcl\_detector/human\_poses: geometry\_msgs/PoseStamped
+ pcl\_detector/human\_clouds: sensor\_msgs/PointCloud2

## Visualize

Run `python scripts/visualization.py` to obtain a 2d heatmap using data from `data/record.txt`.

![](/figure/figure_1.png)

## Customization

Topic names, calibration file and log file can be changed in launch file.
