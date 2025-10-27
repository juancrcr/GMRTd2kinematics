import numpy as np
import matplotlib.pyplot as plt
import math

# Data dari NIM
L1 = 28  # Femur dari NIU: 544528
L2 = 26  # Tibia dari NIF: 60526

def buat_matrix(theta, L):
    """Buat transformation matrix"""
    rad = math.radians(theta)
    return np.array([
        [math.cos(rad), -math.sin(rad), 0, L*math.cos(rad)],
        [math.sin(rad), math.cos(rad), 0, L*math.sin(rad)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def forward_kinematics(theta1, theta2):
    """FK pakai homogeneous matrix"""
    T01 = buat_matrix(theta1, L1)
    T12 = buat_matrix(theta2, L2)
    T02 = np.dot(T01, T12)
    
    x = T02[0, 3]
    y = T02[1, 3]
    
    return x, y, T01, T12, T02

def inverse_kinematics(x_target, y_target):
    """IK untuk cari sudut dari posisi"""
    d = math.sqrt(x_target**2 + y_target**2)
    
    if d > (L1 + L2) or d < abs(L1 - L2):
        return None, None
    
    cos_theta2 = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = math.acos(cos_theta2)
    
    alpha = math.atan2(y_target, x_target)
    beta = math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))
    theta1 = alpha - beta
    
    return math.degrees(theta1), math.degrees(theta2)

def gambar_robot(theta1, theta2):
    """Visualisasi"""
    x, y, _, _, _ = forward_kinematics(theta1, theta2)
    x1 = L1 * math.cos(math.radians(theta1))
    y1 = L1 * math.sin(math.radians(theta1))
    
    plt.figure(figsize=(8, 8))
    plt.plot([0, x1, x], [0, y1, y], 'o-', linewidth=2, markersize=8)
    plt.plot(0, 0, 'ks', markersize=10)
    plt.plot(x, y, 'g^', markersize=10)
    plt.text(x, y+3, f'({x:.1f}, {y:.1f})')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.title(f'θ1={theta1}°, θ2={theta2}°')
    plt.show()

# study case day 2
print("="*40)
print("STUDY CASE")
print("="*40)
print(f"L1 = {L1} mm, L2 = {L2} mm")
print(f"θ1 = 40°, θ2 = 30°\n")

x, y, T01, T12, T02 = forward_kinematics(40, 30)

print("Matrix T01:")
print(T01)
print("\nMatrix T12:")
print(T12)
print("\nMatrix T02:")
print(T02)

print(f"\n{'='*40}")
print(f"JAWABAN: ({x:.2f}, {y:.2f}) mm")
print(f"{'='*40}")
gambar_robot(40, 30)
