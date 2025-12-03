#!/bin/bash
set -e
while getopts t:p:f opt
do
    case $opt in
        t)
            type=$OPTARG
            ;;
        p)
            platform=$OPTARG
            ;;
        f)
            option="--no-cache"
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
    echo "auto select arch:${platform}" 
fi

case $platform in
"arm64")
    platform="arm64"
    ;;
"x86_64"|"amd64")
    platform="amd64"
    ;;
*)
    echo "unknown cpu-arch ${platform}"
    echo "Use 'docker buildx ls' to get supported ARCH"
    exit
    ;;
esac

repository="crpi-6ty60rr45g7d9i1q.cn-shanghai.personal.cr.aliyuncs.com"
namespace="voyance"
packagename="fastlivo2"_$platform

case $type in
    'build')
        docker buildx build $option --platform="linux/"$platform --network=host -t $repository/$namespace/$packagename:ros2 .
        ;;
    'push')
        echo "push to dst registry"
        # ali passwd: Cb1314521*
        # docker login crpi-6ty60rr45g7d9i1q.cn-shanghai.personal.cr.aliyuncs.com --username=gmaui000
        docker push $repository/$namespace/$packagename:ros2
        ;;
     *)
        echo "unkonwn type"
        echo $help_string
        exit
        ;;
esac
