[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME         | BAUDRATE
/dev/ttyUSB_u2d2_0  | 3000000  | l_hip_pitch 
/dev/ttyUSB_u2d2_1  | 3000000  | l_shoulder_pitch

[ device info ]
## TYPE   | PORT NAME			| ID   | MODEL                     | PROTOCOL | DEV NAME           | BULK READ ITEMS
dynamixel | /dev/ttyUSB_u2d2_1	|  1   | ALICE3_PH42-020-S300-R-1  | 2.0      | l_shoulder_pitch   | present_position, present_current
dynamixel | /dev/ttyUSB_u2d2_1	|  2   | ALICE3_PH42-020-S300-R-2  | 2.0      | r_shoulder_pitch   | present_position, present_current

dynamixel | /dev/ttyUSB_u2d2_1	|  3   | ALICE3_PH42-020-S300-R-3  | 2.0      | l_shoulder_roll    | present_position, present_current
dynamixel | /dev/ttyUSB_u2d2_1	|  4   | ALICE3_PH42-020-S300-R-4  | 2.0      | r_shoulder_roll    | present_position, present_current
dynamixel | /dev/ttyUSB_u2d2_1	|  5   | ALICE3_XH540-V270-R-5     | 2.0      | l_elbow_pitch      | present_position, present_current
dynamixel | /dev/ttyUSB_u2d2_1	|  6   | ALICE3_XH540-V270-R-6     | 2.0      | r_elbow_pitch      | present_position, present_current

dynamixel | /dev/ttyUSB_u2d2_1	|  7   | ALICE3_XH540-V270-R-7     | 2.0      | head_pitch         | present_position, present_current
dynamixel | /dev/ttyUSB_u2d2_1	|  8   | ALICE3_XH540-V270-R-8     | 2.0      | head_yaw           | present_position, present_current

dynamixel | /dev/ttyUSB_u2d2_1	|  9   | ALICE3_XH540-V270-R-9     | 2.0      | waist_yaw          | present_position, present_current
dynamixel | /dev/ttyUSB_u2d2_1	|  10  | ALICE3_PH54-100-S500-R-10 | 2.0      | waist_pitch        | present_position, present_current

dynamixel | /dev/ttyUSB_u2d2_0	| 11   | ALICE3_PH54-100-S500-R-11 | 2.0      | l_hip_pitch        | present_position, present_current
dynamixel | /dev/ttyUSB_u2d2_0	| 12   | ALICE3_PH54-100-S500-R-12 | 2.0      | r_hip_pitch        | present_position, present_current
dynamixel | /dev/ttyUSB_u2d2_0	| 13   | ALICE3_PH54-100-S500-R-13 | 2.0      | l_hip_roll         | present_position, present_current
dynamixel | /dev/ttyUSB_u2d2_0	| 14   | ALICE3_PH54-100-S500-R-14 | 2.0      | r_hip_roll         | present_position, present_current
dynamixel | /dev/ttyUSB_u2d2_0	| 15   | ALICE3_XH540-V270-R-15    | 2.0      | l_hip_yaw          | present_position, present_current
dynamixel | /dev/ttyUSB_u2d2_0	| 16   | ALICE3_XH540-V270-R-16    | 2.0      | r_hip_yaw          | present_position, present_current

dynamixel | /dev/ttyUSB_u2d2_0	| 17   | ALICE3_PH54-100-S500-R-17 | 2.0      | l_knee_pitch       | present_position, present_current
dynamixel | /dev/ttyUSB_u2d2_0	| 18   | ALICE3_PH54-100-S500-R-18 | 2.0      | r_knee_pitch       | present_position, present_current
dynamixel | /dev/ttyUSB_u2d2_0	| 19   | ALICE3_PH54-100-S500-R-19 | 2.0      | l_ankle_pitch      | present_position, present_current
dynamixel | /dev/ttyUSB_u2d2_0	| 20   | ALICE3_PH54-100-S500-R-20 | 2.0      | r_ankle_pitch      | present_position, present_current
dynamixel | /dev/ttyUSB_u2d2_0	| 21   | ALICE3_PH54-100-S500-R-21 | 2.0      | l_ankle_roll       | present_position, present_current
dynamixel | /dev/ttyUSB_u2d2_0	| 22   | ALICE3_PH54-100-S500-R-22 | 2.0      | r_ankle_roll       | present_position, present_current