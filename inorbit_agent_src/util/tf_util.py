"""
Useful functions to deal with 3D poses in the form of
geometry_msgs/Transform messages, 7-DOF poses (translation + quaternion)
and 4 by 4 matrices.


NOTE Importing this module requires knowing ROS module locations. Do not
import this module directly. Instead, use ros.py agentlet and call tf_util()
to obtain a reference.

"""
import numpy as np
import transformations

try:
    from geometry_msgs.msg import Transform, Pose
except Exception as e:
    raise Exception("Error importing ROS modules from tf_util; are ROS paths already set?")


def geommsgpose_to_pose(pose):
    """
    Given a geometry_msgs.Pose,
    Returns pose in the following format
    [p_x, p_y, p_z, q_x, q_y, q_z, q_w]
    """

    return np.array(
        [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    )


def pose_to_geommsgpose(pose):
    """
    Input in the following format
    [p_x, p_y, p_z, q_x, q_y, q_z, q_w]

    Returns a geometry_msgs.Pose object with position and orientation.
    """

    p = Pose()
    (
        p.position.x,
        p.position.y,
        p.position.z,
        p.orientation.x,
        p.orientation.y,
        p.orientation.z,
        p.orientation.w,
    ) = pose
    return p


def pose_to_transform(pose):
    """
    Input pose must have format
    [p_x, p_y, p_z, q_x, q_y, q_z, q_w]
    """

    transform = Transform()
    (
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    ) = pose
    return transform


def transform_to_pose(transform):
    """
    Returns pose in the following format
    [p_x, p_y, p_z, q_x, q_y, q_z, q_w]
    """

    return np.array(
        [
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ]
    )


def pose_to_matrix(p):
    """
    Input pose must have format
    [p_x, p_y, p_z, q_x, q_y, q_z, q_w]
    """

    x, y, z, qx, qy, qz, qw = p
    t = transformations.translation_matrix([x, y, z])
    r = transformations.quaternion_matrix([qx, qy, qz, qw])
    return transformations.concatenate_matrices(t, r)


def matrix_to_pose(m):
    """
    Returns pose in the following format
    [p_x, p_y, p_z, q_x, q_y, q_z, q_w]
    """

    (x, y, z) = transformations.translation_from_matrix(m)
    (qx, qy, qz, qw) = transformations.quaternion_from_matrix(m)
    return [x, y, z, qx, qy, qz, qw]


def transform_to_matrix(transform):
    """
    Returns pose in the following format
    [p_x, p_y, p_z, q_x, q_y, q_z, q_w]
    """

    return pose_to_matrix(transform_to_pose(transform))


def matrix_to_transform(matrix):
    """
    Returns transform with the given matrix
    translation and rotation parts.
    """

    return pose_to_transform(matrix_to_pose(matrix))


def substract_transforms(transform_a, transform_b):
    """
    Returns transform c that goes from b to a (as if
    b were the origin of coordinates).
    """

    matrix_a = transform_to_matrix(transform_a)
    matrix_b = transform_to_matrix(transform_b)
    inv_matrix_b = transformations.inverse_matrix(matrix_b)
    matrix_c = transformations.concatenate_matrices(inv_matrix_b, matrix_a)
    return matrix_to_transform(matrix_c)
