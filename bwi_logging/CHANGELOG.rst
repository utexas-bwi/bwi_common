^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bwi_logging
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.12 (2016-09-14)
-------------------

0.3.11 (2016-08-27)
-------------------

0.3.10 (2016-08-15)
-------------------

0.3.9 (2016-08-05)
------------------

0.3.8 (2016-06-06)
------------------
* add some ROS progress logging (`#46
  <https://github.com/utexas-bwi/bwi_common/issues/46>`_)
* do not wait for bags upload to finish (`#46
  <https://github.com/utexas-bwi/bwi_common/issues/46>`_)
* automatically upload any bag with "bwi" prefix (`#46
  <https://github.com/utexas-bwi/bwi_common/issues/46>`_)
* always run rosbag record with -j option (`#46
  <https://github.com/utexas-bwi/bwi_common/issues/46>`_)
* rewrite launch file to conditionally run the node to record extra
  topics
* compute time and distance totals (`#41
  <https://github.com/utexas-bwi/bwi_common/issues/41>`_)
* compress bag data on upload (`#41
  <https://github.com/utexas-bwi/bwi_common/issues/41>`_)
* Drop 99 out of 100 odometry message (`#41
  <https://github.com/utexas-bwi/bwi_common/issues/41>`_)
* create target upload directory, if needed (`#40
  <https://github.com/utexas-bwi/bwi_common/issues/40>`_)
* Contributors: Jack O'Quin, Pato

0.3.7 (2015-09-22)
------------------
* make logging work, even when installed as a binary package (`#32 <https://github.com/jack-oquin/bwi_common/issues/32>`_)
* add upload option to not delete files copied (`#32 <https://github.com/jack-oquin/bwi_common/issues/32>`_)
* add upload script for bags (`#32 <https://github.com/jack-oquin/bwi_common/issues/32>`_)
* add /diagnostics to default topic list (`#32 <https://github.com/jack-oquin/bwi_common/issues/32>`_)
* create logging directory, if needed (`#32 <https://github.com/jack-oquin/bwi_common/issues/32>`_)
* add wrapper node for rosbag_record (`#32 <https://github.com/jack-oquin/bwi_common/issues/32>`_)
* Contributors: Jack O'Quin

0.3.6 (2015-08-25)
------------------

0.3.5 (2015-08-22)
------------------
* bwi_logging: fix catkin lint problems (`#31 <https://github.com/utexas-bwi/bwi_common/issues/31>`_)
* Contributors: Jack O'Quin

0.3.4 (2015-08-19)
------------------
* added maintainers, and minor changes in package.xml files
* merged clingo version 4 updates
* Contributors: Jack O'Quin, Shiqi Zhang

0.3.3 (2015-08-05)
------------------
* edited meta info for bwi_logging
* added bwi_logging package
* Contributors: Shiqi Zhang
