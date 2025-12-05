dirname=${PWD##*/}
username=`whoami`
user_uid=`id -u`
# TODO: fix this so that group id is created if not already in use - especially for mac host
user_gid=1000 #`id -g`

docker build \
  --pull \
  --no-cache \
  --build-arg USERNAME="${username}" \
  --build-arg USER_UID="${user_uid}" \
  --build-arg USER_GID="${user_gid}" \
  -t "${dirname}" .
