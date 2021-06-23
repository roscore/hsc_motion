[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE
/dev/ttyUSB0| 3000000  | l_hip_pitch 


[ device info ]
## TYPE   | PORT NAME    | ID   | MODEL             | PROTOCOL | DEV NAME           | BULK READ ITEMS
dynamixel | /dev/ttyUSB0 |  1   | ALICE1_MX-64-1    | 2.0      | l_shoulder_pitch   | present_position
dynamixel | /dev/ttyUSB0 |  2   | ALICE1_MX-64-2     | 2.0      | r_shoulder_pitch  | present_position

dynamixel | /dev/ttyUSB0 |  3   | ALICE1_MX-106-3    | 2.0      | l_shoulder_roll   | present_position
dynamixel | /dev/ttyUSB0 |  4   | ALICE1_MX-106-4    | 2.0      | r_shoulder_roll   | present_position
dynamixel | /dev/ttyUSB0 |  5   | ALICE1_MX-64-5     | 2.0      | l_elbow_pitch     | present_position
dynamixel | /dev/ttyUSB0 |  6   | ALICE1_MX-64-6     | 2.0      | r_elbow_pitch     | present_position

dynamixel | /dev/ttyUSB0 |  7   | ALICE1_MX-64-7     | 2.0      | head_pitch         | present_position
dynamixel | /dev/ttyUSB0 |  8   | ALICE1_MX-64-8     | 2.0      | head_yaw           | present_position

dynamixel | /dev/ttyUSB0 |  9   | ALICE1_MX-106-9    | 2.0      | waist_yaw          | present_position
#dynamixel | /dev/ttyUSB0 | 10   | ALICE1_MX-106-10   | 2.0      | waist_pitch        | present_position

dynamixel | /dev/ttyUSB0 | 11   | ALICE1_MX-106-11   | 2.0      | l_hip_pitch        | present_position
dynamixel | /dev/ttyUSB0 | 12   | ALICE1_MX-106-12   | 2.0      | r_hip_pitch        | present_position
dynamixel | /dev/ttyUSB0 | 13   | ALICE1_MX-106-13   | 2.0      | l_hip_roll         | present_position
dynamixel | /dev/ttyUSB0 | 14   | ALICE1_MX-106-14   | 2.0      | r_hip_roll         | present_position
dynamixel | /dev/ttyUSB0 | 15   | ALICE1_MX-106-15   | 2.0      | l_hip_yaw          | present_position
dynamixel | /dev/ttyUSB0 | 16   | ALICE1_MX-106-16   | 2.0      | r_hip_yaw          | present_position

dynamixel | /dev/ttyUSB0 | 17   | ALICE1_MX-106-17   | 2.0      | l_knee_pitch       | present_position
dynamixel | /dev/ttyUSB0 | 18   | ALICE1_MX-106-18   | 2.0      | r_knee_pitch       | present_position
dynamixel | /dev/ttyUSB0 | 19   | ALICE1_MX-106-19   | 2.0      | l_ankle_pitch      | present_position
dynamixel | /dev/ttyUSB0 | 20   | ALICE1_MX-106-20   | 2.0      | r_ankle_pitch      | present_position
dynamixel | /dev/ttyUSB0 | 21   | ALICE1_MX-106-21   | 2.0      | l_ankle_roll       | present_position
dynamixel | /dev/ttyUSB0 | 22   | ALICE1_MX-106-22   | 2.0      | r_ankle_roll       | present_position
