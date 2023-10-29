xhost local: root
XAUTH=/tmp/.docker.xauth

sudo docker start sky_ws bash

sudo docker exec -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --privileged \
    sky_ws \
    bash 
