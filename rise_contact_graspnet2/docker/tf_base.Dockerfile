FROM nvidia/cuda:11.0.3-cudnn8-devel-ubuntu20.04
# FROM nvidia/cuda:11.0.3-cudnn8-devel-ubuntu20.04
ARG DEBIAN_FRONTEND=noninteractive

# Nvidia Key
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys A4B469963BF863CC 2>/dev/null

# Change apt source to Kakao mirror
RUN sed -i".bak.original" -re "s/([a-z]{2}.)?archive.ubuntu.com|security.ubuntu.com/mirror.kakao.com/g" /etc/apt/sources.list

# Install essential dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    wget \
    locales \
    curl \
    libgl1-mesa-glx \
    libxrender1 \
    libx11-dev \
    software-properties-common

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1
ENV XDG_RUNTIME_DIR=/tmp

# Tensorflow
RUN pip install --upgrade pip
RUN pip install \
    tensorflow==2.4.0 \
    protobuf==3.20.3 \
    opencv-python==4.4.0.40 \
    pyglet==1.5.29 \
    pyrender==0.1.45 \
    mayavi \
    PyQt5 \
    trimesh \
    scipy \
    pillow \
    tqdm \
    pyyaml

    # mayavi==4.7.3 \
    # PyQt5==5.15.4 \
    # trimesh==3.9.29 \
    # scipy==1.4.1 \


# RUN pip install tensorflow==2.4.0
# RUN pip install --upgrade protobuf==3.20.3

# # OpenCV
# RUN pip3 install PyQt5==5.15.4 \
#     trimesh==3.9.29 \
#     pyrender==0.1.45 \
#     scipy==1.4.1 \
#     protobuf==3.19.6 \
#     pillow \
#     opencv-python==4.4.0.40 \
#     pyglet==1.5.29 \
#     tqdm \
#     pyyaml

RUN apt-get update \
    && apt-get install -y -qq --no-install-recommends \
    # Expose the nvidia driver to allow opengl
    # Dependencies for glvnd and X11.
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    # Additional packages for dev environment
    fonts-firacode \
    tmux \
    tmuxinator \
    xorg-dev \
    zsh \
    net-tools \
    ntpdate \
    freeglut3-dev

# Xvfb
RUN apt-get install -yq --no-install-recommends \
    xvfb \
    x11-utils \
    libx11-dev \
    qt5-default

CMD ["bash"]