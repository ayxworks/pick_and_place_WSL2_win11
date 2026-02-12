#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script para analizar la repetibilidad de poses estimadas
Calcula desviación estándar en posición y orientación
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

def parse_pose_file(filename):
    """
    Lee el archivo de poses y extrae todas las matrices de transformación
    
    Returns:
        list: Lista de matrices 4x4 numpy arrays
    """
    poses = []
    current_matrix = []
    
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            
            # Saltar líneas de encabezado
            if line.startswith('Estimated Pose'):
                if current_matrix:
                    poses.append(np.array(current_matrix))
                    current_matrix = []
                continue
            
            # Procesar líneas con datos numéricos
            if line:
                try:
                    values = [float(x) for x in line.split()]
                    if len(values) == 4:
                        current_matrix.append(values)
                except ValueError:
                    continue
        
        # Agregar la última matriz
        if current_matrix:
            poses.append(np.array(current_matrix))
    
    return poses

def extract_position_orientation(poses):
    """
    Extrae posiciones (x, y, z) y orientaciones (ángulos de Euler) de las poses
    
    Returns:
        positions: array Nx3 con las posiciones
        orientations: array Nx3 con los ángulos de Euler (en grados)
    """
    positions = []
    orientations = []
    
    for pose in poses:
        # Extraer posición (última columna, primeras 3 filas)
        position = pose[:3, 3]
        positions.append(position)
        
        # Extraer matriz de rotación (3x3 superior izquierda)
        rotation_matrix = pose[:3, :3]
        
        # Convertir a ángulos de Euler (XYZ convention)
        rot = R.from_matrix(rotation_matrix)
        euler_angles = rot.as_euler('xyz', degrees=True)
        orientations.append(euler_angles)
    
    return np.array(positions), np.array(orientations)

def calculate_statistics(positions, orientations):
    """
    Calcula estadísticas de repetibilidad
    """
    # Posición: media y desviación estándar
    pos_mean = np.mean(positions, axis=0)
    pos_std = np.std(positions, axis=0)
    pos_std_total = np.linalg.norm(pos_std)  # Desviación total
    
    # Orientación: media y desviación estándar
    ori_mean = np.mean(orientations, axis=0)
    ori_std = np.std(orientations, axis=0)
    ori_std_total = np.linalg.norm(ori_std)  # Desviación total
    
    return pos_mean, pos_std, pos_std_total, ori_mean, ori_std, ori_std_total

def print_results(positions, orientations, pos_mean, pos_std, pos_std_total, 
                  ori_mean, ori_std, ori_std_total):
    """
    Imprime los resultados del análisis
    """
    print("=" * 70)
    print("ANÁLISIS DE REPETIBILIDAD DE POSES")
    print("=" * 70)
    print(f"\nNúmero de poses analizadas: {len(positions)}")
    
    print("\n" + "-" * 70)
    print("POSICIÓN (metros)")
    print("-" * 70)
    print(f"{'Eje':<10} {'Media':<15} {'Desv. Std':<15} {'Rango':<15}")
    print("-" * 70)
    
    axes = ['X', 'Y', 'Z']
    for i, axis in enumerate(axes):
        min_val = np.min(positions[:, i])
        max_val = np.max(positions[:, i])
        rango = max_val - min_val
        print(f"{axis:<10} {pos_mean[i]:<15.6f} {pos_std[i]:<15.6f} {rango:<15.6f}")
    
    print(f"\n{'Total (norma L2):':<25} {pos_std_total:.6f} m")
    
    print("\n" + "-" * 70)
    print("ORIENTACIÓN (grados - Ángulos de Euler XYZ)")
    print("-" * 70)
    print(f"{'Ángulo':<10} {'Media':<15} {'Desv. Std':<15} {'Rango':<15}")
    print("-" * 70)
    
    euler_names = ['Roll (X)', 'Pitch (Y)', 'Yaw (Z)']
    for i, name in enumerate(euler_names):
        min_val = np.min(orientations[:, i])
        max_val = np.max(orientations[:, i])
        rango = max_val - min_val
        print(f"{name:<10} {ori_mean[i]:<15.3f} {ori_std[i]:<15.3f} {rango:<15.3f}")
    
    print(f"\n{'Total (norma L2):':<25} {ori_std_total:.3f}°")
    
    print("\n" + "-" * 70)
    print("DETALLES DE CADA POSE")
    print("-" * 70)
    print(f"{'Pose':<6} {'X':<12} {'Y':<12} {'Z':<12} {'Roll':<10} {'Pitch':<10} {'Yaw':<10}")
    print("-" * 70)
    
    for i in range(len(positions)):
        print(f"{i+1:<6} {positions[i,0]:<12.6f} {positions[i,1]:<12.6f} {positions[i,2]:<12.6f} "
              f"{orientations[i,0]:<10.3f} {orientations[i,1]:<10.3f} {orientations[i,2]:<10.3f}")
    
    print("=" * 70)

def main():
    # Archivo de entrada
    filename = '/home/mgrao/estimated_pose.txt'
    
    # Parsear archivo
    print("Leyendo archivo de poses...")
    poses = parse_pose_file(filename)
    print(f"Se encontraron {len(poses)} poses\n")
    
    # Extraer posiciones y orientaciones
    positions, orientations = extract_position_orientation(poses)
    
    # Calcular estadísticas
    pos_mean, pos_std, pos_std_total, ori_mean, ori_std, ori_std_total = \
        calculate_statistics(positions, orientations)
    
    # Imprimir resultados
    print_results(positions, orientations, pos_mean, pos_std, pos_std_total,
                  ori_mean, ori_std, ori_std_total)

if __name__ == "__main__":
    main()