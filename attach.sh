dirname=${PWD##*/}
container_name=${dirname}_dev

docker exec -it ${container_name} bash