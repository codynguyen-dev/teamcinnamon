dirname=${PWD##*/}
container_name=${dirname}_dev

docker run -it --rm \
  --name ${container_name} \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /mnt/wslg:/mnt/wslg \
  -e DISPLAY \
  -e WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR \
<<<<<<< HEAD
  -v ./src:/student_src \
=======
  -v ./src:/ros_ws/src/student_src \
>>>>>>> template/main
  --privileged \
  ${dirname}