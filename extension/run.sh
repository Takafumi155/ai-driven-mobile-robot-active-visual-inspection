#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
case "${1:-help}" in
  build)
    docker build -t second-look-ros:jazzy .
    docker build -t second-look-imitation:jazzy -f extension/Dockerfile . ;;
  collect|dataset|train|evaluate)
    stage="$1"; shift
    docker run --rm --shm-size=512m -v "$PWD:/work" second-look-imitation:jazzy python3 extension/pipeline.py "$stage" "$@" ;;
  rviz)
    docker run --rm --shm-size=512m -v "$PWD:/work" second-look-imitation:jazzy bash extension/rviz.sh ;;
  deploy)
    shift
    docker run --rm --shm-size=512m -v "$PWD:/work" second-look-imitation:jazzy python3 extension/deploy.py "$@" ;;
  *) echo 'bash extension/run.sh build|collect|dataset|train|evaluate|rviz' ;;
esac
