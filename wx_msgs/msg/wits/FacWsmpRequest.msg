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

# FacWsmpRequest.msg, Fac-WSMP.request 
#
# This message contains information about WSMP TX request.
# There are several enum and bitmask types, a parsing may be necessary as needed.

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
# SEC_TYPE                    = 0x01 << 0
# SIGNER_IDENT_TYPE           = 0x01 << 1
# SOURCE_MAC_ADDRESS          = 0x01 << 2
# RESET_MAC_ADDRESS           = 0x01 << 3
# CHANNEL_IDENTIFIER          = 0x01 << 4
# TIME_SLOT                   = 0x01 << 5
# DATA_RATE                   = 0x01 << 6
# TRANSMIT_POWER_LEVEL        = 0x01 << 7
# USER_PRIORITY               = 0x01 << 8
# EXPIRY_TIME                 = 0x01 << 9
# NOT USED
# NOT USED
# PEER_MAC_ADDRESS            = 0x01 << 12
# PROVIDER_SERVICE_IDENTIFIER = 0x01 << 13

WxHeader wx_header # wx common header

uint32            sec_type                    # sec_type, type: enum
uint32            signer_identifier_type      # signed only
Macaddress48      source_mac_address          # high priority than reset
ChannelIdentifier channel_identifier          # ignore if source mac exists
uint32            time_slot                   # DSRC only, TBD
uint32            flow_slot                   # LTE-V2X only, TBD
uint8             data_rate                   #
int8              transmit_power_level        #
uint8             user_priority               #
uint64            expiry_time                 # 0 means never, epoch, microsec
uint8[]           data                        # payload
Macaddress48      peer_mac_address            #
uint64            provider_service_identifier #
uint16            validity                    #
