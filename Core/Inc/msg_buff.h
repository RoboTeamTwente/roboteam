#include <stdbool.h>

/*
 * create a struct that keeps track of all buffered messages and if they are new or not.
 *  when it is send isNew should be set to false.
 */
struct msgsBufferStatus {
	uint8_t msg[13]; // packet size in bytes
	bool isNew;
};

struct msgsBufferStatus msgBuff[16];
