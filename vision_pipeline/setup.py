from setuptools import find_packages, setup
from glob import glob
import os

package_name = "vision_pipeline"


# Función para obtener todos los archivos de una carpeta recursivamente
def package_files(directory, install_base):
    paths = []
    for path, directories, filenames in os.walk(directory):
        for filename in filenames:
            # Ruta completa del archivo fuente
            file_path = os.path.join(path, filename)
            # Ruta relativa desde el directorio base
            relative_path = os.path.relpath(path, directory)
            # Directorio de instalación destino
            if relative_path == ".":
                install_path = install_base
            else:
                install_path = os.path.join(install_base, relative_path)
            paths.append((install_path, [file_path]))
    return paths


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        # Incluir modelo SAM
        (os.path.join("share", package_name, "models"), glob("vision_pipeline/models/*.pt")),
        # Incluir toda la carpeta weights recursivamente
        *package_files("vision_pipeline/FoundationPose/weights", os.path.join("lib/python3.10/site-packages", package_name, "FoundationPose/weights")),
        # Incluir mycpp
        *package_files("vision_pipeline/FoundationPose/mycpp/build", os.path.join("lib/python3.10/site-packages", package_name, "FoundationPose/mycpp/build")), 
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # Incluir recursos de simulacion
        ( os.path.join("share", package_name, "sim_rsc"), glob("sim_rsc/*.*")), 
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mmujika",
    maintainer_email="mmujika@ikerlan.es",
    description="Vision pipeline based on FoundationPose",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "vision_node = vision_pipeline.vision_node:main",
            "publish_sim_camera_frames = vision_pipeline.publish_sim_camera_frames:main",
            "sim_camera_publisher = vision_pipeline.sim_camera_publisher:main",
        ],
    },
)
