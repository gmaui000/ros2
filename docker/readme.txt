# build the arm64 ros2 container on x64.
~~~
sudo apt update
sudo apt install qemu qemu-user-static
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

bash build_docker_images.sh -t build
bash build_docker_images.sh -t push
~~~


# hub info
~~~
website: hub.docker.com
repo:    ros2
user:    gmaui000
passwd:  cb12345678
~~~
