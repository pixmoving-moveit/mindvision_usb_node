# mindvision_usb_node
The ros node for MindVision Camera

To use this node, follow the steps:
* connected your MindVision camera to a USB 3 port of your computer
* copy the 3rdparty/lib/libMVSDK.so to /lib/
* enter to root
* source devel/setup.bash
* catkin_make
* roslaunch mindvision  mindvision.launch
