import utils

import roboteam_embedded_messages.python.REM_BaseTypes as BaseTypes
from roboteam_embedded_messages.python.REM_RobotMusicCommand import REM_RobotMusicCommand

connection = utils.openContinuous(timeout=0.01)

def cleanRMC():
	cmd = REM_RobotMusicCommand()
	cmd.header = BaseTypes.PACKET_TYPE_REM_ROBOT_MUSIC_COMMAND
	cmd.remVersion = BaseTypes.LOCAL_REM_VERSION
	cmd.id = 12
	return cmd

while True:
	cmd = cleanRMC()

	arg = input("(folder $ song $)(next|previous)(volume up,down,$)(play|pause|stop) Command: ")

	## Song
	if arg.startswith("folder"):
		_, folderId, _, songId = arg.split(" ")
		cmd.folderId = int(folderId)
		cmd.songId = int(songId)
	if arg == "next": cmd.nextSong = 1
	if arg == "previous": cmd.previousSong = 1
	## Volume
	if arg.startswith("volume"):
		subarg = arg.split(" ")[1]
		if subarg == "up": cmd.volumeUp = 1
		elif subarg == "down": cmd.volumeDown = 1
		else: cmd.volume = int(subarg)
	## Mode
	if arg == "play": cmd.play = 1
	if arg == "pause": cmd.pause = 1
	if arg == "stop": cmd.stop = 1

	connection.write(cmd.encode())