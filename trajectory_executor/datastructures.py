serial_read_states = {
  'INIT': 0,
  'READ_PREAMBLE': 1,
  'READ_ROLL': 2,
  'READ_PITCH': 3,
  'READ_YAW': 4,
  'READ_L_HIP': 5,
  'READ_L_KNEE': 6,
  'READ_R_HIP': 7,
  'READ_R_KNEE': 8,
  'READ_L_SHOULDER': 9,
  'READ_L_ELBOW': 10,
  'READ_R_SHOULDER': 11,
  'READ_R_ELBOW': 12,
  'READ_AT_GOAL': 13,
  'READ_CHECKSUM': 14
}

data_packet_struct = {
  'roll': 0,
  'pitch': 0,
  'yaw': 0,
  'l_hip': 0,
  'l_knee': 0,
  'r_hip': 0,
  'r_knee': 0,
  'l_shoulder': 0,
  'l_elbow': 0,
  'r_shoulder': 0,
  'r_elbow': 0,
  'at_goal': 0,
  'checksum': 0,
  'checksum_error': False,
}