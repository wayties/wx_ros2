# Copyright 2020-2021 Wayties, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# FacPotiIndication.msg, Fac-Poti.indication
#
# This message contains periodic information about positioning and timing indication.
# There are several enum and bitmask types, a parsing may be necessary as needed.

WxHeader wx_header # wx common header

# fix_status, type: enum
# NO   = 0
# TIME = 1
# 2D   = 2
# 3D   = 3

# fix_type, type: bitmask          
# position(0)  SINGLE_FREQUENCY   = (1 << 0)
# position(1)  MULTI_FREQUENC     = 
# position(2)  MULTI_CONSTELLATIN =
# position(3)  PPP
# position(4)  INTEGRITY_CHECKED
# position(5)  SBAS
# position(6)  DGNSS
# position(7)  RTK_FIXED
# position(8)  RTK_FLOAT
# position(9)  SSR
# position(10) ESTIMATED
# position(11) DEAD_RECKONING
# position(12) MANUAL
# position(13) SIMULATOR_MODE

# validity, type: bitmask
# position(0)  FIX_STATUS
# position(1)  FIX_TYPE
# position(2)  NUM_USED_SV
# position(3)  NUM_VISIBLE_SV
# position(4)  PDOP
# position(5)  TIMESTAMP
# position(6)  LATITUDE
# position(7)  LONGITUDE
# position(8)  ELEVATION_MSL
# position(9)  ELEVATION_ELLISOID
# position(10) SPEED_GROUND
# position(11) SPEED_VEHICLE
# position(12) HEADING_MOTION
# position(13) HEADING_VEHICLE
# position(14) LONGITUDINAL_ACCELERATION
# position(15) LATERAL_ACCELERATION
# position(16) VERTICAL_ACCELERATION
# position(17) YAW_RATE
# position(18) SEMI_MAJOR_AXIS_ACCURACY
# position(19) SEMI_MINOR_AXIS_ACCURACY
# position(20) SEMI_MAJOR_ORIENTAtion
# position(21) TIME_ACCURACY
# position(22) LATITUDE_ACCURACY
# position(23) LONGITUDE_ACCURACY
# position(24) ELEVATION_ACCURACY
# position(25) SPEED_ACCURACY
# position(26) HEADING_ACCURAC
# position(27) LONGITUDINAL_ACCELERATION_ACCURACY
# position(28) LATTERAL_ACCELERATION_ACCURACY
# position(29) VERTICAL_ACCELERATION_ACCURACY
# position(30) YAW_RATE_ACCURACY

uint32 fix_status #
uint32 fix_type   #

uint16 num_used_sv # 
uint16 num_visible_sv

uint16 p_dop

uint64 timestamp

int32  latitude
int32  longitude
int16  elevation_msl
int16  elevation_ellipsoid 

uint16 speed_ground
uint16 speed_vehicle
uint16 heading_motion
uint16 heading_vehicle

int16  longitudinal_acceleration
int16  lateral_acceleration
int16  vertical_acceleration
int16  yaw_rate

uint8  semi_major_axis_accuracy
uint8  semi_minor_axis_accuracy
uint16 semi_major_orientation

uint32 time_accuracy
uint32 latitude_accuracy
uint32 longitude_accuracy
uint32 elevation_accuracy
uint32 speed_accuracy
uint32 heading_accuracy
uint32 longitudinal_acceleration_accuracy
uint32 latteral_acceleration_accuracy
uint32 vertical_acceleration_accuracy
uint32 yaw_rate_accuracy                             

uint32 validity
