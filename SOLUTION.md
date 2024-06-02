## Docker Image Build (Foxy)

```sh
cd rise_contact_graspnet2/docker/
docker build -t tf_base:latest -f tf_base.Dockerfile .
docker build -t test_ros2:latest -f test_ros2.Dockerfile .
```

## Start Container (Server)

(1) Start Container

git clone 후에는 볼륨 경로 확인할 것.

```sh
docker run --rm -it \
   --name contact_graspnet_server \
   --gpus all \
   --privileged \
   -v /home/sangyoon/Desktop/contact_graspnet_save/rise_contact_graspnet2:/home/ros_ws/src/rise_contact_graspnet2 \
   -v /home/sangyoon/Desktop/contact_graspnet_save/rise_contact_msg2:/home/ros_ws/src/rise_contact_msg2 \
   -v /home/ubuntu/.ros/:/root/.ros/ \
   -v /etc/timezone:/etc/timezone:ro \
   -v /etc/localtime:/etc/localtime:ro \
   test_ros2:latest \
   bash
```

(2) TensorFlow Ops Build (*in container*)

볼륨에 빌드되므로 한번만 하면 됨. 컨테이너를 지우더라도 유지됨.

```sh
cd /home/ros_ws/src/rise_contact_graspnet2
sh compile_pointnet_tfops.sh
```

(3) ROS Build (*in container*)

```sh
cd /home/ros_ws
colcon build --symlink-install --packages-up-to rise_contact_graspnet2
source install/setup.bash
```

(4) Execute : `ros2_node.py 실행`

```sh
ros2 run rise_contact_graspnet2 ros2_node
```

## Test Container

(1) Start Container

git clone 후에는 볼륨 경로 확인할 것.

```sh
docker run --rm -it \
   --name test_client \
   --gpus all \
   --privileged \
   -v /home/sangyoon/Desktop/contact_graspnet_save/rise_contact_graspnet2:/home/ros_ws/src/rise_contact_graspnet2 \
   -v /home/sangyoon/Desktop/contact_graspnet_save/rise_contact_msg2:/home/ros_ws/src/rise_contact_msg2 \
   -v /home/ubuntu/.ros/:/root/.ros/ \
   -v /etc/timezone:/etc/timezone:ro \
   -v /etc/localtime:/etc/localtime:ro \
   test_ros2:latest \
   bash
```

(2) ROS Build (*in container*)

```sh
cd /home/ros_ws
colcon build --symlink-install --packages-up-to rise_contact_graspnet2
source install/setup.bash
```

(3) Execute : `ros2_client_example.py 실행`

```sh
ros2 run rise_contact_graspnet2 client
```