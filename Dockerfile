FROM nvidia/cuda:12.1.0-devel-ubuntu22.04

# Evitar prompts interactivos
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8

# ============================================
# CAPA 1: Sistema base y locales (cambia raramente)
# ============================================
RUN apt update && apt install -y \
    locales \
    curl \
    software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

# ============================================
# CAPA 2: Instalar ROS2 (cambia raramente)
# ============================================
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && \
    apt-get install -y ros-humble-desktop && \
    rm -rf /var/lib/apt/lists/*

# ============================================
# CAPA 3: Herramientas del sistema (cambia raramente)
# ============================================
RUN apt-get update && apt-get install -y \
    python3.10 \
    python3-pip \
    python3-vcstool \
    python3-colcon-common-extensions \
    wget \
    git \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cyclonedds \
    && rm -rf /var/lib/apt/lists/*

# ============================================
# CAPA 4: PyTorch y dependencias pesadas (cambia raramente)
# ============================================
RUN pip install --no-cache-dir \
    torch==2.1.0+cu121 \
    torchvision==0.16.0+cu121 \
    torchaudio==2.1.0 \
    --index-url https://download.pytorch.org/whl/cu121

RUN pip install --no-cache-dir "git+https://github.com/facebookresearch/pytorch3d.git@stable"

COPY ./vision_pipeline/vision_pipeline/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# ============================================
# CAPA 5: librealsense (cambia raramente)
# ============================================
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.realsenseai.com/Debian/librealsense.pgp \
    | tee /etc/apt/keyrings/librealsense.pgp > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/librealsense.list

RUN apt-get update && apt-get install -y \
    librealsense2-utils \
    librealsense2-dev && \
    rm -rf /var/lib/apt/lists/*

# ============================================
# CAPA 6: Inicializar rosdep (cambia raramente)
# ============================================
RUN pip install --no-cache-dir -U rosdep && \
    rosdep init && \
    rosdep update

# ============================================
# CAPA 7: Workspace setup
# ============================================
WORKDIR /ros2_ws
RUN mkdir -p src/vision_pipeline/vision_pipeline

# ============================================
# CAPA 8: Build de foundationpose (solo si cambia vision_pipeline)
# ============================================
COPY ./vision_pipeline/. src/vision_pipeline/

RUN cd src/vision_pipeline/vision_pipeline/ && bash build_foundationpose.sh
# nvdiffrast
ENV TORCH_CUDA_ARCH_LIST="7.0 7.5 8.0 8.6 8.9 9.0+PTX"
ENV FORCE_CUDA="1"
RUN pip3 install --no-cache-dir git+https://github.com/NVlabs/nvdiffrast.git --no-build-isolation

# ============================================
# CAPA 9: Dependencias externas (solo si cambia dependencies.repos)
# ============================================
COPY dependencies.repos .
RUN vcs import src < dependencies.repos

# Instalar dependencias de los paquetes externos
RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    apt update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/* \
"

# ============================================
# CAPA 10: Código fuente (cambia frecuentemente)
# ============================================
# Copiar el resto del código fuente DESPUÉS de las dependencias
COPY . src/

# Reinstalar dependencias solo si el código nuevo las requiere
RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    apt update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/* \
"

# ============================================
# CAPA 11: Build del workspace
# ============================================
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# ============================================
# CAPA 12: Configuración final
# ============================================
RUN apt-get update && \
    apt-get remove -y python3-matplotlib && \
    rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws