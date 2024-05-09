#!/bin/bash
set -e
while getopts t: opt
do
	case $opt in
		t)
			type=$OPTARG
			;;
		?)
			echo "unkonwn"
			exit
			;;
       esac
done

help_string=".sh [-t build|push]"

if [[ ! -n $platform ]];then
	#platform=`arch`
	platform="arm64"
	echo "auto select arch:${platform}" 
fi

case $platform in
"arm64")
	platform="linux/arm64"
	;;
"x86_64"|"amd64")
	platform="linux/amd64"
	;;
*)
	echo "unknown cpu-arch ${platform}"
	echo "Use 'docker buildx ls' to get supported ARCH"
	exit
	;;
esac

namespace="gmaui000"
packagename="ros2"

case $type in
	'build')
	# 以腾讯云账号为例
	docker buildx build --platform=$platform --network=host -t $namespace/$packagename:latest .
		;;
	'push')
		echo "push to dst registry"
		# 以腾讯云账号为例
		docker login --username=gmaui000
		docker push $namespace/$packagename:latest
		;;
 	*)
		echo "unkonwn type"
		echo $help_string
		exit
		;;
esac
