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

# FacxPotiIndication.msg, FACX-POTI.indication
#
# This message contains periodic information about positioning and timing indication.
# There are several enum and bitmask types, a parsing may be necessary as needed.

WxHeader wx_header # wx common header

# fix_status, type: enum
# NO   = 0
# TIME = 1
# 2D   = 2
# 3D   = 3
# MAX  = 4

# fix_type, type: bitmask          
# SINGLE_FREQUENCY   = (0x01 << 0)
# MULTI_FREQUENCY    = (0x01 << 1)
# MULTI_CONSTELLATIN = (0x01 << 2)
# PPP                = (0x01 << 3)
# INTEGRITY_CHECKED  = (0x01 << 4)
# SBAS               = (0x01 << 5)
# DGNSS              = (0x01 << 6)
# RTK_FIXED          = (0x01 << 7)
# RTK_FLOAT          = (0x01 << 8)
# SSR                = (0x01 << 9)
# ESTIMATED          = (0x01 << 10)
# DEAD_RECKONING     = (0x01 << 11)
# MANUAL             = (0x01 << 12)
# SIMULATOR_MODE     = (0x01 << 13)
# MAX                = (0x01 << 14)

# validity, type: bitmask
# FIX_STATUS                         = (0x01 << 0)                 
# FIX_TYPE                           = (0x01 << 1)
# NUM_USED_SV                        = (0x01 << 2)
# NUM_VISIBLE_SV                     = (0x01 << 3)
# PDOP                               = (0x01 << 4)
# TIMESTAMP                          = (0x01 << 5)
# LATITUDE                           = (0x01 << 6)
# LONGITUDE                          = (0x01 << 7)
# ELEVATION_MSL                      = (0x01 << 8)
# ELEVATION_ELLIPSOID                = (0x01 << 9)
# SPEED_GROUND                       = (0x01 << 10)
# SPEED_VEHICLE                      = (0x01 << 11)
# HEADING_MOTION                     = (0x01 << 12)
# HEADING_VEHICLE                    = (0x01 << 13)
# LONGITUDINAL_ACCELERATION          = (0x01 << 14)
# LATERAL_ACCELERATION               = (0x01 << 15)
# VERTICAL_ACCELERATION              = (0x01 << 16)
# YAW_RATE                           = (0x01 << 17)
# SEMI_MAJOR_AXIS_ACCURACY           = (0x01 << 18)
# SEMI_MINOR_AXIS_ACCURACY           = (0x01 << 19)
# SEMI_MAJOR_ORIENTAtion             = (0x01 << 20)
# TIME_ACCURACY                      = (0x01 << 21)
# LATITUDE_ACCURACY                  = (0x01 << 22)
# LONGITUDE_ACCURACY                 = (0x01 << 23)
# ELEVATION_ACCURACY                 = (0x01 << 24)
# SPEED_ACCURACY                     = (0x01 << 25)
# HEADING_ACCURACY                   = (0x01 << 26)
# LONGITUDINAL_ACCELERATION_ACCURACY = (0x01 << 27)
# LATTERAL_ACCELERATION_ACCURACY     = (0x01 << 28)
# VERTICAL_ACCELERATION_ACCURACY     = (0x01 << 29)
# YAW_RATE_ACCURACY                  = (0x01 << 30)
# MAX                                = (0x01 << 31)

uint8  fix_status                         # satellite fix status information, enum type
uint16 fix_type                           # GNSS correction services, bitmask type

uint16 num_used_sv                        # number of satellites currently in use for navigation
uint16 num_visible_sv                     # visible number of satellites 

uint16 p_dop                              # position of DOP, units of 0.01

uint64 timestamp                          # nanosecond, UTC

int32  latitude                           # latitude, units of 1/10 micro degree
int32  longitude                          # longitude, units of 1/10 micro degree
int16  elevation_msl                      # height above mean sea level, units of 10 centimeter 
int16  elevation_ellipsoid                # height above ellipsoid, units of 10 centimeter 

uint16 speed_ground                       # ground speed (2-D), units of 0.02 m/s
uint16 speed_vehicle                      # speed (3-D), units of 0.02 m/s 
uint16 heading_motion                     # heading of motion (2-D), units of 0.0125 degrees
uint16 heading_vehicle                    # heading of vehicle (2-D), units of 0.0125 degrees

int16  longitudinal_acceleration          # along the vehicle longitudinal axis, units of 0.01 m/s^2
int16  lateral_acceleration               # along the vehicle lateral axis, units of 0.01 m/s^2
int16  vertical_acceleration              # along the vehicle vertical axis, units of 0.02 G
int16  yaw_rate                           # vehicle's rotation its vertical axis, units of 0.01 degrees per second 

uint8  semi_major_axis_accuracy           # accuracy of radius (length) of the semi-major axis, units of 0.05 meter
uint8  semi_minor_axis_accuracy           # accuracy of radius of the semi-minor axis, units of 0.05 meter
uint16 semi_major_orientation             # orientate angle of the semi-major axis, units of 0.0054932479 deg

uint32 time_accuracy                      # time accuracy estimate, units of nanosecond 
uint32 latitude_accuracy                  # accuracy of latitudei, units of millimeter
uint32 longitude_accuracy                 # accuracy of longitude, units of millimeter
uint32 elevation_accuracy                 # accuracy of elevation, units of millemeter
uint32 speed_accuracy                     # accuracy of speed, units of millimeter per second
uint32 heading_accuracy                   # accuracy of heading, units of 1e-5 degree 
uint32 longitudinal_acceleration_accuracy # accuracy of longitudinal acceleration, units of millimeter per second
uint32 lateral_acceleration_accuracy      # accuracy of lateral acceleration, units of millimeter per second
uint32 vertical_acceleration_accuracy     # accuracy of vertical acceleration, units of millimeter per second
uint32 yaw_rate_accuracy                  # accuracy of yaw rate, units of millimeter per second

uint32 validity                           # whether the data exists or not, bitmask type
