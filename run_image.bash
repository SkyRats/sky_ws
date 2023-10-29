xhost local: root
XAUTH=/tmp/.docker.xauth

sudo docker run -it \
    --name=sky_ws \
    --env="DISPLAY=$DISPLAY" \
    --volume="/tmp/.x11-unix:/tmp/.x11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XATH" \
    --volume="<absolute/path/to/sky_ws>:/home/sky/sky_ws" \
    --net=host \
    --privileged \
    ghcr.io/skyrats/sky_ws:latest \
    bash 
    
