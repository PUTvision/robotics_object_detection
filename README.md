# Robotics 1 - Camera calibration course

* clone the repository
```
git clone https://github.com/PUTvision/robotics_object_detection.git
```

* go to the repository directory
```
cd robotics_object_detection
```

> **Note:** This conteinter uses `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` by default.

* add docker access to the screen:

```bash
xhost +local:root

XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi
```


## Docker to use with NVIDIA GPU

* build the container
```
docker build -t ros2_detection_gpu ./gpu/ --no-cache
```

* run the container
```
docker run -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --privileged \
    --network=host \
    --gpus all \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --shm-size=1024m \
    --name="ros2_detection_gpu" \
    ros2_detection_gpu \
    bash
```


## Docker to use without NVIDIA GPU (CPU only)

* build the container
```
docker build -t ros2_detection_cpu ./cpu/ --no-cache
```

* run the container
```
docker run -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --privileged \
    --network=host \
    --shm-size=1024m \
    --name="ros2_detection_cpu" \
    ros2_detection_cpu \
    bash
```
