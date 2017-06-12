# roboteam\_robothub

Hi there! Some useful things to know for using the RobotHub package and peripherals

# Getting robothub to work.

## Getting rid of ModemManager

ModemManager is a program installed on Ubuntu by default. Afaik it allows you to dial-in via a serial port. A nasty side-effect of this program is that as soon as a serial port appears it grabs it, fills it with 30 bytes of garbage, and then leaves the port unusable for 30 seconds. Since we have eduroam for our internet needs, we can get rid of this ancient piece of machinery.

`sudo apt-get purge modemmanager`

This command and a reboot should take care of it.

## User rights

On vanilla Ubuntu you can only use the serial port if you are in the dialout group, or something similar. However, by default this is not true. So you need to add yourself to the dialout group (or if that doesn't work, google a bit for a similar solution).

`sudo adduser second_user dialout`

Where `second_user` is your username. This command and a reboot should settle it.

(It also might not. Just keep googling and restarting until you find the right command 😅)

## Using robothub

Connect the base station. If everything is OK, a port will appear at `/dev/ttyACM0`. This is _the_ serial port, altough it disguises itself as a file. With `packet\_tester` you can send packets and test if the packets & acknowledgements are sent and received correctly. With `socat` (see below) you can spoof a serial port for testing without the base station. `robothub` will send RobotCommands into this port if you set the rosparam `robot_output_target` to "serial".

### IMPORTANT NOTE
Make sure ModemManager has been eradicated from
your Ubuntu installation. otherwise, upon serial
connection, it will dump about 30 characters of random bytes
into the connection, screwing up the connection.
ModemManager is a program/service of some sort.
http://askubuntu.com/questions/216114/how-can-i-remove-modem-manager-from-boot
sudo apt-get purge modemmanager + reboot or something should do the trick

http://www.boost.org/doc/libs/1_40_0/doc/html/boost_asio/overview/serial_ports.html

# Useful programs

## system\_test

Sends a sequence of `forward, forward, backward, backward, left, right, left, right, rotate left, rotate right, kick` commands to the robot to execute with 1 second pauses. Useful to check if the robot is still working as before (or at least, I hope. Feel free to improve/modify this).

## packet\_tester

Can be used to:
- Test if roboteam\_msgs/RobotCommand's are converted to robot packets correctly. You can do this with `rosrun roboteam_robothub packet_tester msg`. It will ask you to enter a RobotCommand and then shows the data that is ultimately sent to the robot. Useful if you think the computer is sending bad/malformed/incorrect command to the base station and you want to check a specific command. Example, GoToPos is telling a robot to move forward, but the robot is moving left. With this mode you can check if there's a bug in the "packet generation" code.
- Test if the communication stack is performant enough & if it can handle a continuous stream of packets. Run `rosrun roboteam_robothub packet_tester /dev/ttyACM0`, pick a packet, and be sure to answer yes when it asks you if you want to do a benchmark. If you then press enter a few times it will send 10.000 packets to the base station and display the elapsed time afterwards.
- Test if a single packet comes through properly. Useful if you want to check for specific cases if the ACK works properly, if the connection to the robot works, or how a robot behaves in response to a specific packet. Simply run `rosrun roboteam_robothub packet_tester /dev/ttyACM0` and enter the relevant data.

# Useful commands

###

`socat -d -d pty,raw,echo=0 pty,raw,echo=0`

Opens two fake serial ports (/dev/pts/1 and /dev/pts/2, with 1 and 2 being any number). Useful for testing packet logic. Anything that's written to /dev/pts/1 is instantly sent to /dev/pts/2, and vice versa. If you open up a `screen` instance at each end you have a chat program!!!!

`screen /dev/ttyACM0`

Opens a terminal to a port, in which any byte received is displayed on screen and any key typed is sent into the serial port. `/dev/ttyACM0` can be any serial port (maybe even a file?).

`cat README.md | xxd`

Useful for displaying hex output. README.md can be any filelike thing (but not the base station, didn't work too well.)

`cat < /dev/ttyACM0` or `cat /dev/ttyACM0`

If you just want to see the output of a port or want to clear the stream (e.g. Hans pressed het knopje twice).

`printf "\x00\x01\xff" > /dev/ttyACM0`

Writes only the bytes 0, 1, and 255 to the base station.

# What if...

### The first ACK from the base station seems to take ages?

Hook up the basestation, make sure nothing from ROS is running, and then connect to the port with `cutecom`. Send two or three messages (`////////` or `hhhhhhhh` will do) and close cutecom. If you then unhook the basestation and then start/connect everything as normal, the delay "should" be gone.

### I can only open the serial port once with screen, and then it stops working!

`screen` closes the port when you close screen. Like, not closing like robothub or packet\_tester does it, but really close it. If you need to poke into the serial port before using it and still want to use it afterwards use `cat` and `printf` (or figure out how to keep screen from closing it. Would be interesting to know).

### The base station seems to mess up the ACK's?

Reconnect the base station. Open a terminal. `screen /dev/ttyACM0`. Hit a key 8 times (s or / or any key with a specific ascii code). If the base station responds with two hexadecimal characters it's working as expected. If it doesn't respond like that something weird is going on.

Candidates for "something weird":
- Unplug and plug the base station again, and press "het knopje" again. It's easy to forget (Hans, can you implement a LED that lights up if it is initialized?).
- Sometimes the software of the main pcb is accidentally uploaded to the base station, and vice versa. This is Bad™. Contact your local electrical engineer.
- The programmer cable of the base station is still plugged in, while you have the communication USB cable plugged in. Ask Hans to unplug the programmer cable.
- The communication USB cable is not plugged in.
- Did you remove modemmanager?
- Did you press "het knopje"?
- Did you?

### The base station always ACKs with "re" and then (optionally) seems to stop working?

Sometimes when Hans presses "het knopje" the base station prints debug output to USB. These are about 40 lines with the form "register 0 = a", "register 1 = b", etc. So when robothub tries to receive an ACK, instead of the two hexadecimal numbers it receives the first two letters of "register", which are "r" and "e". If for some reason this cannot be disabled or Hans is not around, you can work around this by running `cat /dev/ttyACM0` after "het knopje" was pressed, which empties the USB buffer to the base station. From that on the ACKs should be working properly.

The reason why often the base station seems to stop working when this happens (and optionally starts blinking, if Hans didn't remove that code) is because as soon as robothub receives "re", it sends another packet. Since the base station is still busy handling the previous message(s), the buffer is overflowed and the base station locks up.

### This is all wayyyy to convoluted? Can't I just send a bunch of bytes to the base station and read any ACK that comes this way?

Sure. Open a terminal and run `cat /dev/ttyACM0`. This will print any bytes that are sent by the base station. Open another terminal and run `printf "\x01\x23\x34\x56\x78\x9a\xbc\xde" > /dev/ttyACM0`. The second terminal will then send 8 bytes to the base station (each \x## is a byte; \x00 is the 0 byte and \xff is 255). This can occasionally be useful if you can't get packet\_tester/system\_test/robothub to do what you want.

### We can only send 10-100 messages per second

The link between the computer and the base station is fairly fast, i.e. less than 1 ms per packet is achieved if the base station doesn't to anything else besides receiving them. So if you experience a packet limit then there's either something slow happening in your program, the base station is broken, or some bad software was uploaded to the base station. (There used to be some 10ms sleeps in the base station/main pcb software at some point, so maybe they snuck back in?)

### I want to let robothub send its robotcommands into a port made with socat and print the packets out as soon as they enter the port, but it only prints garbage. How do I format the output like a human readable packet?

No clue, I didn't write a tool for that. Sounds like a good idea :p

### The base station cannot send anything to the main pcb (i.e. the base station always responds with a NACK)!

Are you specificying the right robot ID in your robot packets? At the time of writing this only the ID 7 is used because it is obviously the best number. The other numbers are fake.

Besides that, the main pcb might be turned off, have the incorrect software uploaded to it, or the NRF chip might be badly connected/broken. A cause that we had earlier was that the main pcb crashed as soon as the motors started turning, causing only the first packet to be ACK'd and all the subsequent packets to be NACK'd. Hope that that's not the case anymore :o
