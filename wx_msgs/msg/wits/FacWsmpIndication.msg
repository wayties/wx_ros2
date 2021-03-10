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

# FacWsmpIndication.msg, Fac-WSMP.indication 
#
# This message contains information about WSMP RX indication.
# There are several enum and bitmask types, a parsing may be necessary as needed.

WxHeader wx_header # wx commonn header

# sec_type, type: enum
# UNSECURED = 0
# SIGNED    = 1

# signer_identifier_type, type: enum
# CERTIFICATE = 0
# DIGEST      = 1

# provider_service_identifier
# Default set is 0x7F for Testing.
# If need to Vehicle to Vehicle Safety and Awareness, change to 0x20

uint32       sec_type                    # unsecured only
uint32       signer_identifier_type      # signed only
Macaddress48 peer_mac_address            # generally use broadcast address ff:ff:ff:ff:ff:ff
uint64       provider_service_identifier # default: 0x7F
uint8[]      data                        # payload
