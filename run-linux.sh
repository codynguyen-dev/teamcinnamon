dirname=${PWD##*/}
container_name=${dirname}_dev

docker run -it --rm \
  --name ${container_name} \
  --env="DISPLAY=host.docker.internal:0" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --env="XAUTHORITY=$XAUTH" \
  --volume="$XAUTH:$XAUTH" \
  -v ./src:/ros_ws/src/student_src \
  --privileged \
${dirname}