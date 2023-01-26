#!/usr/bin/env python

import numpy as np
from scipy import linalg

def quat_to_rot_matrix(q):
    # Input must be in the form [w,x,y,z]
    q = np.array(q, dtype=float)
    q = q / np.linalg.norm(q)
    qw, qx, qy, qz = q
    rot_matrix = np.array([
        [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw, 0],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw, 0],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2, 0],
        [0, 0, 0, 1]
    ])
    return rot_matrix

def translation_vector_to_matrix(t):
    t = np.array(t, dtype=float)
    return np.array([
        [1, 0, 0, t[0]],
        [0, 1, 0, t[1]],
        [0, 0, 1, t[2]],
        [0, 0, 0, 1]
    ])

def matrix_to_translation_quat(matrix):
    translation_vector = matrix[:3, 3]
    rot_matrix = matrix[:3, :3]
    quat = np.empty((4, ), dtype=np.float32)

    m00, m01, m02, m03 = matrix[0]
    m10, m11, m12, m13 = matrix[1]
    m20, m21, m22, m23 = matrix[2]

    tr = m00 + m11 + m22
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        quat[0] = 0.25 * S
        quat[1] = (m21 - m12) / S
        quat[2] = (m02 - m20) / S
        quat[3] = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = np.sqrt(1.0 + m00 - m11 - m22) * 2
        quat[0] = (m21 - m12) / S
        quat[1] = 0.25 * S
        quat[2] = (m01 + m10) / S
        quat[3] = (m02 + m20) / S
    elif m11 > m22:
        S = np.sqrt(1.0 + m11 - m00 - m22) * 2
        quat[0] = (m02 - m20) / S
        quat[1] = (m01 + m10) / S
        quat[2] = 0.25 * S
        quat[3] = (m12 + m21) / S
    else:
        S = np.sqrt(1.0 + m22 - m00 - m11) * 2
        quat[0] = (m10 - m01) / S
        quat[1] = (m02 + m20) / S
        quat[2] = (m12 + m21) / S
        quat[3] = 0.25 * S
    return translation_vector, quat

if __name__ == '__main__':
    quat = [0.707, 0.707, 0, 0]
    quat2 = [0.5, 0.5, -0.5,0.5]
    p = quat_to_rot_matrix(quat)
    q = quat_to_rot_matrix(quat2)
    r = np.matmul(p,q)
    print(r)