#!/bin/bash

. $(dirname "$0")/color_variables.sh

while true
do
    bin/gripper-interface $@
    sleep 0.1
done
