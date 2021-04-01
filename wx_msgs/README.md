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
* [FacxPotiIndication](msg/wits/FacxPotiIndication.msg): Periodic indication message having information for Global Navigation Satellite System and Inertial measurement unit 
* [FacxWsmpRequest](msg/wits/FacxWsmpRequest.msg): Request message used to send a WSM packet  
* [FacxWsmpConfirm](msg/wits/FacxWsmpConfirm.msg): Confirm message received in response to whether FacWsmpRequest has been properly transmitted or not 
* [FacxWsmpIndication](msg/wits/FacxWsmpIndication.msg): Indication message for a received WSM packet 

## Services (.srv)
* TBD 

## Quality Declaration
* TBD
