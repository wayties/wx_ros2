# wx_msgs

This package provides many messages and services relating to Wayties V2X (WX) devices.

For more information about ROS 2 interfaces, see [index.ros2.org](https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/)

## Messages (.msg)

### - Header
* [WxHeader](msg/WxHeader.msg): WX common metadata for stamped data types used to communicate timestamped data in a data frame

### - IEEE
* [ChannelIdentifier](msg/ieee/ChannelIdentifier.msg): Type message used to indicate a fully specified channel, comprising Country String, Operating Class, and Channel Number 
* [Macaddress48](msg/ieee/Macaddress48.msg): Type message representing MAC address following EUI-48

### - WITS
* [FacPotiIndication](msg/wits/FacPotiIndication.msg): Periodic indication message having information for Global Navigation Satellite System and Inertial measurement unit 
* [FacWsmpRequest](msg/wits/FacWsmpRequest.msg): Request message used to send a WSM packet  
* [FacWsmpConfirm](msg/wits/FacWsmpConfirm.msg): Confirm message received in response to whether FacWsmpRequest has been properly transmitted or not 
* [FacWsmpIndication](msg/wits/FacWsmpIndication.msg): Indication message for a received WSM packet 

## Services (.srv)
* TBD 

## Quality Declaration
* TBD
