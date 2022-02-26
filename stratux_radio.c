#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>

#include "uat.h"
#include "fec.h"

//#include <stdio.h>
//#include <fcntl.h>
//#include <string.h>
//#include <unistd.h>
//#include "fec.h"

unsigned char radioMagic []= {0x0a, 0xb0, 0xcd, 0xe0};
unsigned char buf[1024];
unsigned char tmpBuf[200];
unsigned char toRelay[2000];
unsigned char to[1000];

void radioSerialPortReader(FILE *fd);
void processRadioMessage(int st, int ed) ;

int main(int argc, char* argv[])
{
    //char radio_fifo[] = "/tmp/radio";
    char radio_log[] = "/home/pi/PhidgetInsurments/mag_dataadsb_log.txt";
    //FILE *fd = fopen(radio_fifo, O_RDONLY);
    // from py code file = open('/home/pi/PhidgetInsurments/mag_dataadsb_log.txt', 'ab')
    FILE *fd = fopen(radio_log, O_RDONLY);
    radioSerialPortReader(fd);
}

void radioSerialPortReader(FILE *fd) {
	
	int head = 0;
    int bufLen = 0;
    int numMessages = 0;
    int msgLen = 0;
	while(1){

        size_t rc = fread(tmpBuf, sizeof(tmpBuf), 1, fd);

		//buf = append(buf, tmpBuf[:n]...)
        memccpy(&buf[head], &tmpBuf[0], rc , sizeof(char));
        head = head + rc;
		bufLen = head;
		
        //var finBuf []byte   // Truncated buffer, with processed messages extracted.
		//var numMessages int // Number of messages extracted.

		// Search for a suitable message to extract.
        
		for (int i = 0; i < bufLen-6; i++)
        {
			if ((buf[i] == radioMagic[0]) && (buf[i+1] == radioMagic[1]) && 
                (buf[i+2] == radioMagic[2]) && (buf[i+3] == radioMagic[3])) 
            {
				// Found the magic sequence. Get the length.
				msgLen = (__uint16_t)(buf[i+4]) + ((__uint16_t)(buf[i+5])<<8) + 5 ;// 5 bytes for RSSI and TS.

				// Check if we've read enough to finish this message.
				if (bufLen < i+6+msgLen) {
					break; // Wait for more of the message to come in.
				}
				// Message is long enough.
				//processRadioMessage(buf[i+6 : i+6+msgLen])
                processRadioMessage(i+6, i+6+msgLen);

				// Remove everything in the buffer before this message.
				//finBuf = buf[i+6+msgLen:]
                head = i+6+msgLen;
				numMessages += 1;
			}
		}
		//if (numMessages > 0) {
		//	buf = finBuf
		//}
	}
}

/*
	processRadioMessage().
	 Processes a single message from the radio. De-interleaves (when necessary), checks Reed-Solomon, passes to main process.
*/

void processRadioMessage(int st, int ed) {
	// RSSI and message timestamp are prepended to the actual packet.

	// RSSI
	unsigned char rssiRaw = (buf[st]);
	//rssiAdjusted := int16(rssiRaw) - 132 // -132 dBm, calculated minimum RSSI.
	//rssiDump978 := int16(1000 * (10 ^ (float64(rssiAdjusted) / 20)))
	int rssiDump978 = (int)rssiRaw;

	//_ := uint32(msg[1]) + (uint32(msg[2]) << 8) + (uint32(msg[3]) << 16) + (uint32(msg[4]) << 24) // Timestamp. Currently unused.

	//msg = msg[5:]
    //int msglen = sizeof(msg)-5;
    int msglen = ed-st-5;

	int rs_errors = 0;

	switch (msglen) 
    {
	case 552:
		correct_uplink_frame(&buf[st], to, &rs_errors);
        to[432]=0;
		int cnt = sprintf((char*)toRelay, "+%s;ss=%d;", (char*)to, rssiDump978);
        toRelay[cnt+1] = 0;
	case 48:
		//copy(to, msg);
        memccpy(&to, &buf[st], ed-st , sizeof(char));
		int i = correct_adsb_frame((unsigned char*)to, &rs_errors);
		if (i == 1) {
			// Short ADS-B frame.
            to[18]=0;
			int cnt = sprintf((char*)toRelay, "-%s;ss=%d;", (char*)to, rssiDump978);
            toRelay[cnt+1] = 0;
		} 
        else if (i == 2) {
			// Long ADS-B frame.
            to[34]=0;
			cnt= sprintf((char*)toRelay, "-%s;ss=%d;",  (char*)to, rssiDump978);
            toRelay[cnt+1] = 0; 
		}

	//default:
		//print("processRadioMessage(): unhandled message size \n", msglen);
	}

	if (strlen((char*)toRelay) > 0 && rs_errors != 9999) {
		// send to frame pipe
	}
}