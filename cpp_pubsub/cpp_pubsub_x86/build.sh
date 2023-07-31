#!/bin/bash
source /opt/tros/setup.bash

colcon build --packages-select cpp_pubsub
