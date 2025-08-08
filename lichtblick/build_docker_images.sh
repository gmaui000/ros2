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

repository="crpi-6ty60rr45g7d9i1q.cn-shanghai.personal.cr.aliyuncs.com"
namespace="voyance"
packagename="lichtblick"

case $type in
    'build')
        if [ ! -e "./lichtblick" ]; then
            git clone https://github.com/lichtblick-suite/lichtblick.git
            cd lichtblick
        else
            cd lichtblick && git pull
        fi
        docker buildx build --platform=$platform --network=host -t $repository/$namespace/$packagename:latest .
        ;;
    'push')
        echo "push to dst registry"
        # docker login --username=gmaui000
        docker push $repository/$namespace/$packagename:latest
        ;;
     *)
        echo "unkonwn type"
        echo $help_string
        exit
        ;;
esac
