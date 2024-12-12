## Docker Image Build (Foxy)

```sh
cd rise_contact_graspnet2/docker/
docker build -t tensorflow_base:latest -f tensorflow_base.Dockerfile .
docker build -t contact_grasp_ros2ws:latest -f contact_grasp_ros2ws.Dockerfile .
```

## Start Container (Server)

(1) Start Container

git clone 후에는 볼륨 경로 확인할 것.

```sh
export CONTACT_GRASPNET_DIR=$WS_DIR/src/server/rise-contact_graspnet

docker run --rm -it \
   --name contact_graspnet_server \
   --gpus all \
   --privileged \
   -v $CONTACT_GRASPNET_DIR/rise_contact_graspnet2:/home/ros_ws/src/rise_contact_graspnet2 \
   -v $CONTACT_GRASPNET_DIR/rise_contact_msg2:/home/ros_ws/src/rise_contact_msg2 \
   -v /home/ubuntu/.ros/:/root/.ros/ \
   -v /etc/timezone:/etc/timezone:ro \
   -v /etc/localtime:/etc/localtime:ro \
   contact_grasp_ros2ws:latest \
   bash
```

(2) TensorFlow Ops Build (*in container*)

볼륨에 빌드되므로 한번만 하면 됨. 컨테이너를 지우더라도 유지됨.

```sh
cd /home/ros_ws/src/rise_contact_graspnet2
sh compile_pointnet_tfops.sh
```

이 단계의 결과로 다음의 파일들이 생성됨. (그래서 컨테이너를 지우더라도 유지됨.)

```sh
$CONTACT_GRASPNET_DIR/rise_contact_graspnet2/pointnet2/tf_ops/sampling/tf_sampling_so.so
$CONTACT_GRASPNET_DIR/rise_contact_graspnet2/pointnet2/tf_ops/sampling/tf_sampling_g.cu.o
$CONTACT_GRASPNET_DIR/rise_contact_graspnet2/pointnet2/tf_ops/sampling/1.pkl
$CONTACT_GRASPNET_DIR/rise_contact_graspnet2/pointnet2/tf_ops/grouping/tf_grouping_so.so
$CONTACT_GRASPNET_DIR/rise_contact_graspnet2/pointnet2/tf_ops/grouping/tf_grouping_g.cu.o
```

(3) ROS Build (*in container*)

```sh
cd /home/ros_ws
colcon build --symlink-install --packages-up-to rise_contact_graspnet2
source install/setup.bash
```

(4) Execute : `ros2_node.py 실행`

아래 노드를 실행하기 전에, 모델 checkpoint 파일을 공식 홈페이지에서 다운로드 받아야 한다. (https://github.com/NVlabs/contact_graspnet?tab=readme-ov-file#download-models-and-data)

```sh
ros2 run rise_contact_graspnet2 ros2_node
```

## Test Container

(1) Start Container

git clone 후에는 볼륨 경로 확인할 것.

```sh
export CONTACT_GRASPNET_DIR=$WS_DIR/src/server/rise-contact_graspnet

docker run --rm -it \
   --name test_client \
   --gpus all \
   --privileged \
   -v $CONTACT_GRASPNET_DIR/rise_contact_graspnet2:/home/ros_ws/src/rise_contact_graspnet2 \
   -v $CONTACT_GRASPNET_DIR/rise_contact_msg2:/home/ros_ws/src/rise_contact_msg2 \
   -v /home/ubuntu/.ros/:/root/.ros/ \
   -v /etc/timezone:/etc/timezone:ro \
   -v /etc/localtime:/etc/localtime:ro \
   contact_grasp_ros2ws:latest \
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