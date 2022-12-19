### RoboTeam Basestation
# udev rules
For linux users, place the file [`99-platformio-udev.rules`](https://docs.platformio.org/en/latest/faq.html#platformio-udev-rules) in the folder `/etc/udev/rules.d`. This will give Platformio the right permissions to open a connection to the Basestation (specifically, USB devices with vendorId 0483).
# dialout group	
Linux users have to add themselves to the dialout group : `sudo usermod -a -G dialout $USER`