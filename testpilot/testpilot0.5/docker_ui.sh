#!/bin/bash
xhost +
docker run -t -d --name=testpilot_$2 --network=testpilot-net -p 5559-5561 --rm --ip=$1 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix testpilot
