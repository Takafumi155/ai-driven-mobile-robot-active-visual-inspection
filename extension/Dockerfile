FROM second-look-ros:jazzy
RUN apt-get update && apt-get install -y --no-install-recommends ros-jazzy-rviz2 ros-jazzy-rosbag2 python3-pyqt5 && rm -rf /var/lib/apt/lists/*
ENV PYTHONPATH=/work QT_X11_NO_MITSHM=1
CMD ["python3", "extension/pipeline.py", "--help"]
