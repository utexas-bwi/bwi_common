^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bwi_rqt_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.7 (2015-09-22)
------------------

0.3.6 (2015-08-25)
------------------
* bwi_rqt_plugins: remove broken CMake install (`#34 <https://github.com/utexas-bwi/bwi_common/issues/34>`_)
* Contributors: Jack O'Quin

0.3.5 (2015-08-22)
------------------
* bwi_rqt_plugins: fix catkin lint problems (`#31 <https://github.com/utexas-bwi/bwi_common/issues/31>`_)
* Contributors: Jack O'Quin

0.3.4 (2015-08-19)
------------------
* change segbot references to bwi_common (fixes `#29 <https://github.com/utexas-bwi/bwi_common/issues/29>`_)
* Contributors: Jack O'Quin

0.3.3 (2015-08-05)
------------------
* fix catkin warning about bwi_rqt_plugins headers
* wrap buttons around if too many.
* switched controls from qshortcut to key press/release events for faster response time.
* fixed image_view plugin to use settings for image topic name, since rqt cpp plugins don't support ros parametrization.
* adding a separate package for bwi_rqt_plugins.
* Contributors: Jack O'Quin, Piyush Khandelwal
