FROM nvidia/cuda:12.1.0-devel-ubuntu22.04

# Evitar prompts interactivos
ENV DEBIAN_FRONTEND=noninteractive

# Install ROS2 Humble
RUN apt-get update && apt install -y software-properties-common && add-apt-repository universe 
RUN apt install curl -y
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
RUN dpkg -i /tmp/ros2-apt-source.deb
RUN apt-get update
RUN apt-get install -y ros-humble-desktop


# Instalar herramientas necesarias
RUN apt-get update && apt-get install -y \
    python3-vcstool \
    wget \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/lists/*

# Instalar python
RUN apt update && apt install -y python3.10 python3-pip

# Install torch
RUN pip install torchvision==0.16.0+cu121 torchaudio==2.1.0 torch==2.1.0+cu121 --index-url https://download.pytorch.org/whl/cu121
RUN pip install "git+https://github.com/facebookresearch/pytorch3d.git@stable"


# Crear workspace
WORKDIR /ros2_ws

RUN mkdir -p src/vision_pipeline/vision_pipeline

# Preparar sistema de build de foundationpose
COPY ./vision_pipeline/. src/vision_pipeline/
RUN cd src/vision_pipeline/vision_pipeline/ && bash build_foundationpose.sh

# Clone and install nvdiffrast
ENV TORCH_CUDA_ARCH_LIST="7.0 7.5 8.0 8.6 8.9 9.0+PTX"
ENV FORCE_CUDA="1"
RUN pip3 install --no-cache-dir git+https://github.com/NVlabs/nvdiffrast.git --no-build-isolation

# Copiar src y fichero de dependencias
COPY . src/
COPY dependencies.repos .

# Importar dependencias
RUN vcs import src < src/dependencies.repos

# Librealsense
# Instalar llave y repositorio de Intel RealSense
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.realsenseai.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" \
        | tee /etc/apt/sources.list.d/librealsense.list && \
    apt-get update

# Instalar librealsense2 y herramientas
RUN apt-get update && apt-get install -y \
    librealsense2-utils \
    librealsense2-dev && \
    rm -rf /var/lib/apt/lists/*

# Inicializar rosdep
RUN pip install -U rosdep
RUN rosdep init && rosdep update


# Instalar dependencias del workspace
RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    apt update &&\ 
    rosdep install --from-paths src --ignore-src -r -y \
"
CMD ["bash"]
# Source de ROS + build
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Source automático al entrar al contenedor
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

RUN apt remove -y python3-matplotlib

# Directorio por defecto
WORKDIR /ros2_ws
