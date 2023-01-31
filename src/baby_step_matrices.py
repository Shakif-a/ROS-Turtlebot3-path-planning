#!/usr/bin/env python

import rospy
import numpy as np
from scipy import linalg
import geometry_msgs.msg
from geometry_msgs.msg import Vector3, Quaternion
import tf2_ros
from fiducial_msgs.msg import FiducialTransformArray
from hear_fiducial import vel_manipulator

def quat_to_rot_matrix(quat1):
    # Input must be in the form [w,x,y,z]
    quat1 = np.array(quat1, dtype=float)
    quat1 = quat1 / np.linalg.norm(quat1)
    qw, qx, qy, qz = quat1
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
        # outputs in the form [x y z] and [w x y z]
    return translation_vector, quat

def extract_variables(obj):
    return [obj.w, obj.x, obj.y, obj.z] if hasattr(obj, 'w') else [obj.x, obj.y, obj.z]

def tf_broadcaster(broadcast, link, parent_frame, child_frame, translation, rotation):
		
		# Function to broacast tf's
		link.header.stamp = rospy.Time.now()
		link.header.frame_id = parent_frame
		link.child_frame_id = child_frame

		link.transform.translation = translation

		link.transform.rotation = rotation
		broadcast.sendTransform(link)

		return

def calculations(translation, quaternion):
    # Specific function to calculate inverse of fiducial transform, and go to cam link using lookup
    trans_input = extract_variables(translation)
    quat_input = extract_variables(quaternion)
    # lookup would be added later: using constants for now
    constant_quat = geometry_msgs.msg.Quaternion(0.497,-0.504,0.496,0.503)
    constant_trans = geometry_msgs.msg.Vector3(0.015,0.0,0.0)
    trans_const = extract_variables(constant_trans)
    quat_const = extract_variables(constant_quat)
    trans_const_matrix = translation_vector_to_matrix(trans_const)
    quat_const_matrix = quat_to_rot_matrix(quat_const)
    # initial matrices
    trans_input_matrix = translation_vector_to_matrix(trans_input)
    quat_input_matrix = quat_to_rot_matrix(quat_input)
    mat_fiducial = np.matmul(trans_input_matrix,quat_input_matrix)
    mat_forward = np.matmul(trans_const_matrix, quat_const_matrix)
    mat_inv = linalg.inv(mat_fiducial)
    mat_final = np.matmul(mat_forward, mat_inv)
    return mat_final

    
if __name__ == '__main__':
    rospy.init_node('my_static_tf2_broadcaster')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    base_link = geometry_msgs.msg.TransformStamped()
    
    quat1 = [0.49, 0.4, 0.31, -0.71]
    quat = Quaternion(quat1[1],quat1[2],quat1[3],quat1[0])

    tran1 = [1.0, 2.0, 3.0]
    tran = Vector3(tran1[0],tran1[1],tran1[2])

    r = quat_to_rot_matrix(quat1)
    d = translation_vector_to_matrix(tran1)
    new = np.matmul(d,r)

    new = linalg.inv(new)

    (d2,r2) = matrix_to_translation_quat(new)
    print(r2)
    print(d2)
    tf_broadcaster(broadcaster, base_link, "map", "turtle", tran, quat)
    rospy.sleep(2)
    r2 = Quaternion(r2[1],r2[2],r2[3],r2[0])
    d2 = Vector3(d2[0],d2[1],d2[2])
    tf_broadcaster(broadcaster, base_link, "turtle", "beetle", d2, r2)
    rospy.sleep(2)
    constant_quat = Quaternion(0.497,-0.504,0.496,0.503)
    constant_trans = Vector3(0.51,0.0,0.0)

    cqm = extract_variables(constant_quat)
    ctm = extract_variables(constant_trans)
    cqm = quat_to_rot_matrix(cqm)
    ctm = translation_vector_to_matrix(ctm)
    new2 = np.matmul(ctm,cqm)
    (d3,r3) = matrix_to_translation_quat(new2)
    r3 = Quaternion(r3[1],r3[2],r3[3],r3[0])
    d3 = Vector3(d3[0],d3[1],d3[2])
    tf_broadcaster(broadcaster, base_link, "beetle", "honeybee", d3, r3)
    new3 = np.matmul(new,new2)
    rospy.sleep(2)

    (d4,r4) = matrix_to_translation_quat(new3)
    r4 = Quaternion(r4[1],r4[2],r4[3],r4[0])
    d4 = Vector3(d4[0],d4[1],d4[2])
    tf_broadcaster(broadcaster, base_link, "turtle", "soup", d4, r4)

    rospy.loginfo("Done")