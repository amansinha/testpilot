#!/bin/bash
docker ps --filter name=testpilot -aq | xargs docker kill