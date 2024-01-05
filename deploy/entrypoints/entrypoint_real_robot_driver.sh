#!/bin/bash

set -e

function use() {
    source $1
    echo source $1 >> /root/.bashrc
}

use /opt/ros/humble/setup.bash
use install/local_setup.bash

exec $@