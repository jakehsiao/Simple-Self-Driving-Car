import numpy as np

def get_rotation_matrix(quaternion):
    # given the quaternion, output the rotation matrix
    w = quaternion[0]
    x = quaternion[1]
    y = quaternion[2]
    z = quaternion[3]

    x2 = x ** 2
    y2 = y ** 2
    z2 = z ** 2
    xy = x * y
    yz = y * z
    xz = x * z
    xw = x * w
    yw = y * w
    zw = z * w

    rotation_matrix = np.array(
        [
            [1-2*(y2+z2), 2*(xy-zw), 2*(xz+yw)],
            [2*(xy+zw), 1-2*(x2+z2), 2*(yz-xw)],
            [2*(xz-yw), 2*(yz+xw), 1-2*(x2+y2)],
        ]
    )
    return rotation_matrix

def rotate(position, quaternion):
    return np.dot(get_rotation_matrix(quaternion), np.array([position]).T)
