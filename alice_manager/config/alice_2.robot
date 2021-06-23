[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE
/dev/ttyU2D20 | 3000000  | l_shoulder_pitch
/dev/ttyU2D21 | 3000000  | l_hip_pitch
/dev/ttyU2D22 | 3000000  | r_hip_pitch


[ device info ]
## TYPE   | PORT NAME     | ID   | MODEL                | PROTOCOL | DEV NAME           | BULK READ ITEMS
dynamixel | /dev/ttyU2D20 |  1   | ALICE2_MX-106-1    | 2.0      | l_shoulder_pitch  | present_position
dynamixel | /dev/ttyU2D20 |  2   | ALICE2_MX-106-2    | 2.0      | r_shoulder_pitch  | present_position

dynamixel | /dev/ttyU2D20 |  3   | ALICE2_MX-106-3     | 2.0      | l_shoulder_roll   | present_position
dynamixel | /dev/ttyU2D20 |  4   | ALICE2_MX-106-4     | 2.0      | r_shoulder_roll   | present_position
dynamixel | /dev/ttyU2D20 |  5   | ALICE2_MX-64-5      | 2.0      | l_elbow_pitch     | present_position
dynamixel | /dev/ttyU2D20 |  6   | ALICE2_MX-64-6      | 2.0      | r_elbow_pitch     | present_position

dynamixel | /dev/ttyU2D20 |  7   | ALICE2_MX-64-7      | 2.0      | head_pitch         | present_position
dynamixel | /dev/ttyU2D20 |  8   | ALICE2_MX-64-8      | 2.0      | head_yaw           | present_position

#dynamixel | /dev/ttyU2D20 |  9   | ALICE2_MX-106-9     | 2.0      | waist_yaw          | present_position
#dynamixel | /dev/ttyU2D20 | 10   | ALICE2_MX-106-10    | 2.0      | waist_pitch        | present_position

dynamixel | /dev/ttyU2D21 | 11   | ALICE2_H54P-100-S500-R-11   | 2.0      | l_hip_pitch        | present_position
dynamixel | /dev/ttyU2D22 | 12   | ALICE2_H54P-100-S500-R-12   | 2.0      | r_hip_pitch        | present_position
dynamixel | /dev/ttyU2D21 | 13   | ALICE2_H54P-100-S500-R-13   | 2.0      | l_hip_roll         | present_position
dynamixel | /dev/ttyU2D22 | 14   | ALICE2_H54P-100-S500-R-14   | 2.0      | r_hip_roll         | present_position
dynamixel | /dev/ttyU2D21 | 15   | ALICE2_XH540-V270-15        | 2.0      | l_hip_yaw          | present_position
dynamixel | /dev/ttyU2D22 | 16   | ALICE2_XH540-V270-16        | 2.0      | r_hip_yaw          | present_position

dynamixel | /dev/ttyU2D21 | 17   | ALICE2_H54P-100-S500-R-17   | 2.0      | l_knee_pitch       | present_position
dynamixel | /dev/ttyU2D22 | 18   | ALICE2_H54P-100-S500-R-18   | 2.0      | r_knee_pitch       | present_position
dynamixel | /dev/ttyU2D21 | 19   | ALICE2_H54P-100-S500-R-19   | 2.0      | l_ankle_pitch      | present_position
dynamixel | /dev/ttyU2D22 | 20   | ALICE2_H54P-100-S500-R-20   | 2.0      | r_ankle_pitch      | present_position
dynamixel | /dev/ttyU2D21 | 21   | ALICE2_H54P-100-S500-R-21   | 2.0      | l_ankle_roll       | present_position
dynamixel | /dev/ttyU2D22 | 22   | ALICE2_H54P-100-S500-R-22   | 2.0      | r_ankle_roll       | present_position
