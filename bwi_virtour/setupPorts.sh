#!/bin/bash
ssh -L 9090:localhost:9090 http://nixons-head.csres.utexas.edu # rosbridge
ssh -L 8080:localhost:8080 http://nixons-head.csres.utexas.edu # video stream
ssh -L 8000:localhost:8000 http://nixons-head.csres.utexas.edu # image server
