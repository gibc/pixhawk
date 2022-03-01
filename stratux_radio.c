#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>

#define UAT_H
#include "uat.h"
#include "fec.h"
#undef UAT_DECODE_H
#include "uat_decode.h"





unsigned char radioMagic []= {0x0a, 0xb0, 0xcd, 0xe0};
unsigned char buf[1024];
unsigned char swpBuf[1024];
unsigned char tmpBuf[200];
unsigned char toRelay[200];
unsigned char to[1000];

int findMagic(FILE* fd);
int getMsg(FILE* fd);
void radioSerialPortReader(FILE *fd);
void processRadioMessage(int st, int ed) ;
char radio_log[] = "/home/pi/PhidgetInsurments/mag_dataadsb_log.txt";


int bad_msg_count = 0;
int good_msg_count = 0;

int main(int argc, char* argv[])
{
    //char radio_fifo[] = "/tmp/radio";
    //char radio_log[] = "/home/pi/PhidgetInsurments/mag_dataadsb_log.txt";
    //FILE *fd = fopen(radio_fifo, O_RDONLY);
    // from py code file = open('/home/pi/PhidgetInsurments/mag_dataadsb_log.txt', 'ab')
	printf("start radio2frame\n");
 
	

	init_fec();
	FILE *fd;
	//int a = 10;
    fd = fopen(radio_log, "r"); //home/pi/PhidgetInsurments/mag_dataadsb_log.txt
	
    radioSerialPortReader(fd);

	
	fclose(fd);
	printf("close connection bad msg cnt: %d good msg cnt %d\n", bad_msg_count, good_msg_count);
}

void radioSerialPortReader(FILE *fd) {
	
	int head = 0;
    int bufLen = 0;
    int numMessages = 0;
    int msgLen = 0;
	while(1){

		if (findMagic(fd) < 0)
			break;

		msgLen = getMsg(fd);
		if(msgLen < -1)
			continue;
		if(msgLen < 0)
			break;

		processRadioMessage(0, msgLen);
		//good_msg_count += 1;
		
		memset(buf,0,sizeof buf);

		continue;

        size_t rc = fread(tmpBuf, 1, 200, fd);

		//buf = append(buf, tmpBuf[:n]...)
        memcpy(&buf[head], &tmpBuf[0], rc );
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
		if (numMessages > 0) {
			//clearBuf(head);
			head = 0;
			bufLen = 0;
    		numMessages = 0;
    		msgLen = 0;
		}
	}
}

int getMsg(FILE* fd)
{
	int rc;
	int msgLen; 
	unsigned char tmp[2];
	rc = fread(&tmp[0], 1, 2, fd);
	if (rc < 2)
		return -1;
	
	msgLen = (__uint16_t)(tmp[0]) + ((__uint16_t)(tmp[1])<<8) + 5;
	if(msgLen > 800){
		int pos = ftell(fd);
		bad_msg_count += 1;
		return -2;
	}

	
	rc = fread(&buf[0], 1, msgLen, fd);
	if (rc < msgLen)
		return -1;

	buf[msgLen] = 0;
	
	return msgLen;

}

// find magic 4 bytes
int findMagic(FILE* fd)
{
	unsigned char buf[10];
	int cnt = 0;

	while ( fread(&buf[0], 1, 1, fd) > 0)
	{
		if (buf[0] == radioMagic[cnt]){
        	cnt = cnt + 1;
        	if (cnt >= 4)
            	return ftell(fd);
		}
		else
		{
			cnt = 0;
			continue;
		}
	}  
	return -1;
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
    int msglen = (ed-st)-5;

	int rs_errors = 0;

	switch (msglen) 
    {
	case 552:
		correct_uplink_frame(&buf[st+5], to, &rs_errors);
        to[432]=0;
		int cnt = sprintf((char*)toRelay, "+%s;ss=%d;", (char*)to, rssiDump978);
        toRelay[cnt+1] = 0;
	case 48:
		//copy(to, msg);
        memcpy(&to[0], &buf[st+5], msglen );
		int i = correct_adsb_frame((unsigned char*)to, &rs_errors);
		if (i == 1) {
			// Short ADS-B frame.
            to[18]=0;

			good_msg_count += 1;

			struct uat_adsb_mdb mdb_zero; 
			
    		
			uat_decode_adsb_mdb((char*)to, &mdb_zero);
			if(mdb_zero.callsign[0] == 0)
			{
				mdb_zero.callsign[0] = 'e';
				mdb_zero.callsign[1] + 'm';
				mdb_zero.callsign[2] = 'p';
				mdb_zero.callsign[3] + 't';
				mdb_zero.callsign[4] = 'y';
				
			}

			printf("icao: %d call sign: %s lat: %f lon: %f alt: %d track: %d speed: %d vert speed %d\n", 
					mdb_zero.address, 
					&mdb_zero.callsign ,
					mdb_zero.lat, 
					mdb_zero.lon,
					mdb_zero.altitude,
					mdb_zero.track,
					mdb_zero.speed,
					mdb_zero.vert_rate
					);
			int cnt = sprintf((char*)toRelay, "-%s;ss=%d;", (char*)to, rssiDump978);
            toRelay[cnt+1] = 0;
		} 
        else if (i == 2) {
			// Long ADS-B frame.
            to[34]=0;
			cnt= sprintf((char*)toRelay, "-%s;ss=%d;",  (char*)to, rssiDump978);
            toRelay[cnt+1] = 0; 
		}
		else{
			bad_msg_count += 1;
		}

	//default:
		//print("processRadioMessage(): unhandled message size \n", msglen);
	}

	//if (strlen((char*)toRelay) > 0 && rs_errors != 9999) {
	if (0) {
		unsigned char tb[50];
		memset(tb,0,sizeof tb);

		int len = 0;
		len = strlen(toRelay);
		for(int i = 0; i < len; i++)
		{
			printf("%d ", toRelay[i]);
		}
		printf("\n");
		//fflush(stdout);
		//sprintf(tb, "%s", toRelay[0]);

		//printf("frame: %s\n", toRelay);
		//int i = 0;
		//fflush(stdout);
	}
}