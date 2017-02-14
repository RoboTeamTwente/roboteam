# roboteam_robothub

Hi there!

# Getting robothub to work.

## Getting rid of ModemManager

ModemManager is a program installed on Ubuntu by default. Afaik it allows you to dial-in via a serial port. A nasty side-effect of this program is that as soon as a serial port appears it grabs it, fills it with 30 bytes of garbage, and then leaves the port unusable for 30 seconds. Since we have eduroam for our internet needs, we can get rid of this ancient piece of machinery.

`sudo apt-get purge modemmanager`

This command and a reboot should take care of it.

## User rights

On vanilla Ubuntu you can only use the serial port if you are in the dialout group, or something similar. However, by default this is not true. So you need to add yourself to the dialout group (or if that doesn't work, google a bit for a similar solution).

`sudo adduser second_user dialout`

Where `second_user` is your username. This command and a reboot should settle it.

## Using robothub

Connect the base station. If everything is OK, a port will appear at `/dev/ttyACM0`. This is _the_ serial port, altough it disguises itself as a file. With packet tester you can send packets and test if the packets & acknowledgements are sent and received correctly. With `socat` (see below) you can spoof a serial port for testing without the base station.

# Useful commands

`socat -d -d pty,raw,echo=0 pty,raw,echo=0`

Opens two fake serial ports (/dev/pts/1 and /dev/pts/2, with 1 and 2 being any number). Useful for testing packet logic.

`screen /dev/ttyACM0`

Opens a terminal to a port, in which any byte received is displayed on screen and any key typed is sent into the serial port. `/dev/ttyACM0` can be any serial port (maybe even a file?).

`cat README.md | xxd`

Useful for displaying hex output.

`cat < /dev/ttyACM0`

If you just want to see the output of a port.


