# serial_node for eurobot 2021

- Download dependent package :
http://wjwwood.io/serial/

- Before you run the nodes:

    1. Enter below commands
    
        ```
        $ echo 'KERNEL=="ttyUSB*", KERNELS=="1-1.3", MODE:="0777", GROUP:="dialout", SYMLINK+="STM1"' >> /etc/udev/rules.d/usb.rules
        
        $ echo 'KERNEL=="ttyUSB*", KERNELS=="1-1.4", MODE:="0777", GROUP:="dialout", SYMLINK+="STM2"' >> /etc/udev/rules.d/usb.rules

        $ sudo service udev reload

        $ sudo service udev restart

        -> the output will tell you to enter something, just follow it. Next, you will have to enter the restart command again.

        $ sudo service udev restart
    
    2. Unplug any USB devices and then plug them back to RPI.

    3. Congrats! The node should work properly. If you have any questions pls ask Wei-Chieh.
 
