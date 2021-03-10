## WX-ROS2
ROS 2 Interface for Wayties V2X (WX)

### Prerequisites 
ROS 2 Foxy Fitzroy [[ROS2 Foxy Install](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html)], Ubuntu 20.04
###  
### Clone & Build
```
git clone https://github.com/wayties/wx_ros2.git
cd wx_ros2
colcon build --symlink-install
```

### Getting Started 
Open a new terminal, navigate to `ws_ros2`, and source the setup files:
```
source install/local_setup.bash
```

Now run the ROS2 WsmpPoti application: 
```
$ ros2 run wsmp_poti wsmp_poti
[INFO] [1615358883.854047800] [WsmpPoti]: [SUB][FAC-POTI-IND] nts: 1615358883.853653669, seq: 111
[INFO] [1615358883.854164600] [WsmpPoti]:                     Fix Status (valid)  : 3 (0: NO, 1: TIME, 2: 2D FIX, 3: 3D FIX)
[INFO] [1615358883.854234000] [WsmpPoti]:                     Used SV    (invalid): 0 
[INFO] [1615358883.854299600] [WsmpPoti]:                     Latitude   (valid)  :  37.4110107 [deg]
[INFO] [1615358883.854416900] [WsmpPoti]:                     Longitude  (valid)  : 127.0954984 [deg]
[INFO] [1615358883.854442600] [WsmpPoti]:                     Elevation  (valid)  :       112.3 [meter]
[INFO] [1615358883.854563400] [WsmpPoti]:                     Speed      (valid)  :      109.64 [m/s]
[INFO] [1615358883.854702500] [WsmpPoti]: [SUB][FAC-WSMP-IND] nts: 1615358883.854152203, seq: 111
[INFO] [1615358883.854767900] [WsmpPoti]:                     WSM payload size: 46
[INFO] [1615358883.910554900] [WsmpPoti]: [PUB][FAC-WSMP-REQ] nts: 1615358883.910467863, seq: 123
[INFO] [1615358883.910654100] [WsmpPoti]:                     WSM Payload Size: 43
[INFO] [1615358883.910745800] [WsmpPoti]:                     User Priority   : 5
[INFO] [1615358883.910805900] [WsmpPoti]:                     PSID            : 0x7F
[INFO] [1615358883.911364400] [WsmpPoti]: [SUB][FAC-WSMP-CFM] nts: 1615358883.910467863, seq: 123
[INFO] [1615358883.911426700] [WsmpPoti]:                     Result Code: ACCEPTED
```
If there is no device, you can test using the emulator:
```
$ ros2 run wsmp_poti_emu ros2 run wsmp_poti_emu
[INFO] [1615358883.853735500] [WsmpPotiEmu]: [PUB][FAC-POTI-IND] nts: 1615358883.853653669, seq: 111
[INFO] [1615358883.853842300] [WsmpPotiEmu]:                     Fix Status: 3 (0: NO, 1: TIME, 2: 2D FIX, 3: 3D FIX)
[INFO] [1615358883.853875700] [WsmpPotiEmu]:                     Latitude :  37.4110107 [deg]
[INFO] [1615358883.853936900] [WsmpPotiEmu]:                     Longitude: 127.0954984 [deg]
[INFO] [1615358883.853999900] [WsmpPotiEmu]:                     Elevation:       112.3 [meter]
[INFO] [1615358883.854067300] [WsmpPotiEmu]:                     Speed    :      109.64 [m/s]
[INFO] [1615358883.854307400] [WsmpPotiEmu]: [PUB][FAC-WSMP-IND] nts: 1615358883.854152203, seq: 111
[INFO] [1615358883.854421300] [WsmpPotiEmu]:                     WSM payload size: 46
[INFO] [1615358883.910868200] [WsmpPotiEmu]: [SUB][FAC-WSMP-REQ] nts: 1615358883.910467863, seq: 123
[INFO] [1615358883.910949100] [WsmpPotiEmu]:                     WSM payload size: 43
[INFO] [1615358883.910999900] [WsmpPotiEmu]:                     User Priority (valid)  : 5
[INFO] [1615358883.911049100] [WsmpPotiEmu]:                     PSID          (valid)  : 0x7F
[INFO] [1615358883.911132100] [WsmpPotiEmu]: [PUB][FAC-WSMP-CFM] nts: 1615358883.910467863, seq: 123
[INFO] [1615358883.911189100] [WsmpPotiEmu]:                     Result Code: ACCEPTED
```
FAC-POTI-IND and FAC-WSMP-IND are simplex indication. \
But, FAC-WSMP-CFM is a response of FAC-WSMP-REQ.
```
+----------+                          +-----------------+  
|          |  <--- FAC-POTI-IND ----  |                 | 
|          |                          |                 |
|   ROS2   |  <--- FAC-WSMP-CFM ------------     WX     |
|          |                          |    ^   Device   |
| WsmpPoti |                          |    |     or     |  
|          |  ---- FAC-WSMP-REQ ----------->  Emulator  |
|          |                          |                 |
|          |  <--- FAC-WSMP-IND ----  |                 |
+----------+                          +-----------------+
```