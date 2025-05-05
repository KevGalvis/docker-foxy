#!/bin/bash
# Setup script for ROS2 workspace with Jetson integration

# Update your CycloneDDS configuration with correct Jetson IP
read -p "Enter your Jetson Nano IP address: " jetson_ip
sed -i "s/192.168.1.6/$jetson_ip/g" workspace/cyclonedds_config.xml

# Start the Docker container
docker-compose up -d

# Connect to the container
docker exec -it ros_foxy_gui bash -c "
echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc && 
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc && 
echo 'export CYCLONEDDS_URI=/workspace/cyclonedds_config.xml' >> ~/.bashrc && 
cd /workspace && 
mkdir -p src && 
echo 'cd /workspace' >> ~/.bashrc && 
bash"

echo "Container started and configured. Now run VS Code:"
echo "code ~/Docker-foxy"
echo "Then reopen in container."