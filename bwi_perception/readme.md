# bwi_perception

* Tabletop scene detection
* Plane detectors

## Naming

_Perception_ actions and services take input directly from the camera. They aggregate many pointclouds and perform detection on the result. They expose simple filtering (e.g. region bounding) parameters. These services are meant for expedience.

_Detection_ actions and services operate on a pointcloud provided by the caller. They expect filtering to be
handled by the caller.