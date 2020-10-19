#!/usr/bin/env python2

import os
from tf import transformations as tfs
from math import radians

def generate_calibration():
    device = raw_input('device >> ')
    x = 0.001*float(raw_input('x >> '))
    y = 0.001*float(raw_input('y >> '))
    z = 0.001*float(raw_input('z >> '))
    o = radians(float(raw_input('o >> ')))
    a = radians(float(raw_input('a >> ')))
    t = radians(float(raw_input('t >> ')))

    if device == 'magnet':
        T = [[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]]
    elif device == 'gripper':
        T = [[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, 0], [0, 0, 0, 1]]
    else:
        T = tfs.identity_matrix()
    T = tfs.concatenate_matrices(tfs.euler_matrix(o, a, t, 'rzyz'), T)

    if device == 'camera':
        dir_path = os.environ['HOME'] + '/.ros/aist_handeye_calibration'
        child    = 'calibrated_' + device
    else:
        dir_path = os.environ['HOME'] + '/.ros/aist_tool_calibration'
        child    = device + '_link'
    filename = dir_path + '/' + device + '.yaml'

    if not os.path.isdir(dir_path):
        os.makedirs(dir_path)

    with open(filename, mode='w') as f:
        q = tfs.quaternion_from_matrix(T)
        s = 'parent: arm_link6\n' \
            + 'child: {}_link\n'.format(device) \
            + 'transform: {{x: {}, y: {}, z: {}, qx: {}, qy: {}, qz: {}, qw: {}}}' \
            .format(x, y, z, q[0], q[1], q[2], q[3])
        if device == 'camera':
            f.write('eye_on_hand: true\n')
        f.write(s)

if __name__ == "__main__":
    try:
        generate_calibration()
    except ValueError:
        print('Non-numeric value!')
    except Exception as e:
        print()
