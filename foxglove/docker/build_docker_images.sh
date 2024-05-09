#!/bin/bash
SHELL_FOLDER=$(cd "$(dirname "$0")";pwd)
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
	platform=`arch`
	#platform="arm64"
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
packagename="foxglove"

case $type in
	'build')
		if [ ! -e "./studio" ]; then
			git clone https://github.com/foxglove/studio.git
			cd studio
		else
			cd studio && git pull
		fi
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
