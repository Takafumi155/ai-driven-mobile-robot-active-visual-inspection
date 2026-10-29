FROM ros:jazzy-ros-base
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image \
    python3-numpy python3-sklearn python3-pil python3-imageio python3-scipy \
    ffmpeg libgl1-mesa-dri libegl1 xvfb xauth && rm -rf /var/lib/apt/lists/*
WORKDIR /work
COPY . /work
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV GZ_SIM_RESOURCE_PATH=/work
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["python3", "runner.py", "--help"]
ENV OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 SECOND_LOOK_ROOT=/work
