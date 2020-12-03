#!/bin/bash
did=$(docker ps -l | cut -d " " -f1|grep -v CONTAINER)
docker exec -it $did /bin/bash