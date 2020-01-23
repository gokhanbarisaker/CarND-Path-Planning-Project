#!/usr/bin/env bash

docker build --no-cache --rm -f "Project.dockerfile" -t carnd-path-planning-project:latest . && docker run -p 4567:4567 --rm -it carnd-path-planning-project:latest
