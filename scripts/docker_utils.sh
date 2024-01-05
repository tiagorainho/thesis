#!/bin/bash

function docker_container_name() {
    local res=$(dig -x `ifconfig eth0 | grep 'inet' | awk '{print $2}'` +short | cut -d'.' -f1)
    echo $res
}

# get container name from dns lookup of the ip of the container
export CONTAINER_NAME=$(docker_container_name)