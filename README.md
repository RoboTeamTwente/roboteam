# roboteam\_robothub

Hi there! Some useful things to know for using the RobotHub package and peripherals

## User rights

On vanilla Ubuntu you can only use the serial port if you are in the dialout group, or something similar. However, by default this is not true. So you need to add yourself to the dialout group (or if that doesn't work, google a bit for a similar solution. For arch this group is callec uucp instead).

`sudo adduser $USER dialout`

This command and a reboot should settle it.

(It also might not. Just keep googling and restarting until you find the right command 😅)

# udev rules
For linux users, place the file [`99-platformio-udev.rules`](https://docs.platformio.org/en/latest/faq.html#platformio-udev-rules) in the folder `/etc/udev/rules.d`, and reboot your pc. This will give robothub the right permissions to open a connection to the basestation (specifically, USB devices with vendorId 0483). Note that this has nothing to do with Platformio in particular. It's just that this file provided by Platformio does the job.