FROM ros:humble

# Evitar prompts interactivos
ENV DEBIAN_FRONTEND=noninteractive

# Instalar herramientas necesarias
RUN apt-get update && apt-get install -y \
    python3-vcstool \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/lists/*

# Crear workspace
WORKDIR /ros2_ws

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


# Instalar dependencias del workspace
RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    sudo apt update &&\ 
    rosdep install --from-paths src --ignore-src -r -y \
"

# Source de ROS + build
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Source automático al entrar al contenedor
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Directorio por defecto
WORKDIR /ros2_ws
