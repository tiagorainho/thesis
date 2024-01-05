#!/bin/bash

set -e

function use() {
    source $1
    echo source $1 >> /root/.bashrc
}

use /scripts/docker_utils.sh
use /scripts/simulation_setup.sh
use /opt/ros/humble/setup.bash
use install/local_setup.bash
exec $@