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

# FacxWsmpRequest.msg, FACX-WSMP.request 
#
# This message contains information about WSMP TX request.
# There are several enum and bitmask types, a parsing may be necessary as needed.
# Almost configurations for RF can be set from www. 

WxHeader wx_header # wx common header

# sec_type, type: enum
# UNSECURED = 0
# SIGNED    = 1

# signer_identifier_type {
# CERTIFICATE = 0,
# DIGESET     = 1

# time_slot, type: enum
# SLOT_0 = 0
# SLOT_1 = 1
# EITHER = 2

# flow_slot, type: enum
# FLOW_SPS = 0
# FLOW_EVT = 1

# validity, type: bitmask
# SEC_TYPE                    = (0x01 << 0)
# SIGNER_IDENT_TYPE           = (0x01 << 1)
# SOURCE_MAC_ADDRESS          = (0x01 << 2)
# CHANNEL_IDENTIFIER          = (0x01 << 3)
# TIME_SLOT                   = (0x01 << 4)
# FLOW_SLOT                   = (0x01 << 5)
# DATA_RATE                   = (0x01 << 6)
# TRANSMIT_POWER_LEVEL        = (0x01 << 7)
# USER_PRIORITY               = (0x01 << 8)
# EXPIRY_TIME                 = (0x01 << 9)
# PEER_MAC_ADDRESS            = (0x01 << 10)
# PROVIDER_SERVICE_IDENTIFIER = (0x01 << 11)
# MAX                         = (0x01 << 12)
# LENGTH NOT USED
# DATA   NOT USED

# provider_service_identifier
# Default set is 0x7F for Testing.
# If need to Vehicle to Vehicle Safety and Awareness, change to 0x20

uint8             sec_type                    # unsecured
uint8             signer_identifier_type      # signed only
Macaddress48      source_mac_address          # individual address for MAC  
ChannelIdentifier channel_identifier          # a fully specified channel info
uint8             time_slot                   # DSRC only, TBD
uint8             flow_slot                   # LTE-V2X only, TBD
uint8             data_rate                   # subject to limitations of the PHY (1 ~ 63.5 Mb/s, 500kbit/s increment) 
int8              transmit_power_level        # TBD
uint8             user_priority               # default: 5 (5: no critical, 7: critical), range(0..7)
uint64            expiry_time                 # 0 means never, epoch, microsec
Macaddress48      peer_mac_address            # generally use broadcast address ff:ff:ff:ff:ff:ff 
uint64            provider_service_identifier # default: 0x7F
uint16            validity                    # whether the data exists or not, bitmask type 
uint8[]           data                        # payload
