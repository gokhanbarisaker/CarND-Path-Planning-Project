#!/usr/bin/env bash

docker ps --quiet | awk '{print $1 }' | xargs -I {} docker kill {}

