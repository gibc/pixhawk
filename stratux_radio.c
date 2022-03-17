#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>

#include <errno.h>
#include <termios.h>
#include <unistd.h>

#define UAT_H
#include "uat.h"
#include "fec.h"
#undef UAT_DECODE_H
#include "uat_decode.h"

//unsigned char radioMagic []= {0x0a, 0xb0, 0xcd, 0xe0};
//unsigned char buf[1024];
//unsigned char swpBuf[1024];
//unsigned char tmpBuf[200];
//unsigned char toRelay[200];
unsigned char to[1000];
//unsigned char readBuf[100];


//int findMagic(FILE* fd);
//int getMsg(FILE* fd);
//void radioSerialPortReader(FILE *fd);
//void processRadioMessage(int pipe, int msg_len, unsigned char* msg) ;
void send_result(int pipe, struct uat_adsb_mdb* mdb_zero);
void uat_display_sv(const struct uat_adsb_mdb *mdb);
//char radio_log[] = "/home/pi/PhidgetInsurments/mag_dataadsb_log.txt";
char serial_path[] = "/dev/serial/by-id/usb-Stratux_Stratux_UATRadio_v1.0_DO0271Z9-if00-port0";//, baudrate=2000000, timeout=5)

int bad_msg_count = 0;
int good_msg_count = 0;

//#define MSG_LENGTH 400

//FILE *fss;
int main(int argc, char* argv[])
{
    //char radio_fifo[] = "/tmp/radio";
    //char radio_log[] = "/home/pi/PhidgetInsurments/mag_dataadsb_log.txt";
    //FILE *fd = fopen(radio_fifo, O_RDONLY);
    // from py code file = open('/home/pi/PhidgetInsurments/mag_dataadsb_log.txt', 'ab')
	printf("start radio2frame\n");

	/*int rfd = open(serial_path, O_RDONLY);
	if(rfd < 0)
	{
		printf("open radio usb falied\n");
	}
	else
	{
		printf("opened radio usb ok\n");
		printf("radio fd: %d\n ", rfd);
		printf("reading from radio usb\n");
		unsigned char lb[10];
		int rc = read(rfd, lb, 5);
		printf("read: %d bytes from radio usb\n", rc);
		perror("Error: ");
	}*/
 
	init_fec();
	//FILE *fd;
	//FILE *fss;

	printf("opening pipe for read\n");
	int in_fp = open("/tmp/send_radio", O_RDONLY) ;
	if(in_fp < 0)
	{
		printf("open in pipe failed\n");
		return 0;
	}

	printf("opening pipe for write\n");
	int out_fp = open("/tmp/receive_radio", O_WRONLY) ;
	//fss = fopen("/tmp/receive_radio", "w");
	if(out_fp < 0)
	{
		printf("open out pipe failed\n");
		return 0;
	}
	printf("opened pipe in_fp: %d O_RDONLY\n", in_fp);
	//printf("opened pipe out_fp: %d O_WRONLY\n", out_fp);

	//unsigned char msg_buf [MSG_LENGTH];
	//unsigned char byts_buf [MSG_LENGTH/4];
	//unsigned char len_buf[1];
	unsigned char int_buf[150];
	while(1)
	{   
		int lc = read(in_fp, int_buf, sizeof(unsigned char));
		printf("lc: %d\n", lc);
		if(lc <= 0)
		{
			return 0;
		}
		printf("array len: %d\n", int_buf[0]);
		//memset(msg_buf,0, sizeof(msg_buf));
		int byte_len = int_buf[0]*sizeof(unsigned char)+1;
		printf("byte len: %d\n", byte_len);
		int cnt = read(in_fp, &int_buf[1], byte_len);
		printf("read %d cnt\n", cnt);
		while(cnt+1 < byte_len)
		{
			int gap = cnt+1 - byte_len;
			printf("less than full read %d more bytes\n", gap);
			int bc = read(in_fp, int_buf[cnt+1], gap);
			printf("read %d more bytes\n", bc);
			cnt = cnt + bc;
			printf("new cnt %d \n", cnt);
		}

		/*for(int i = 0; i < byte_len; i++)
		{
			// printf("buf index: %d: ", i);
			//int val = hex2int(&msg_buf[i]);
			//byts_buf[i/2] = val;
			//printf("out buf %d set to %d\n", i/4, val);
			printf("read val: %d idx: %d \n", int_buf[i], i);
		}*/
		//printf("exit hex2int loop\n");

		//printf("length of received message: %d: \n", byts_buf[0]);
		
		processRadioMessage(out_fp, int_buf[0], &int_buf[1]);
		//printf("return from processRadioMessage\n");
		
	}

		/*if(rd_count >= 10)
		{
			for(int j = 0; j<10; j++)
			{
				printf(" atoi val: %d,", outb[j]);
				printf("\n");
			}
			rd_count = 0;
		}
		else rd_count += 1;

		int val = strtoul(nb, (char**)NULL, 16);
		outb[rd_count] = val;
		printf("val %d ", val);
		//int ml = buf[0];
		//printf("msg len: %d bytes\n", ml);
		//int rb = read(STDIN_FILENO, buf, 10);
		//printf("read cnt: %d\n", rb);
		for(int i = 0; i<cnt; i++)
		{
			printf("got byte %d ", nb[i]);
		}
		printf("\n");
	}
	
    //fd = fopen(radio_log, "r"); //home/pi/PhidgetInsurments/mag_dataadsb_log.txt
	
	//fss = fopen(serial_path, "rb");
	
	//int cnt = fread(&readBuf[0], 1, 1, fss);
	
	/*
	int fs = open(serial_path, O_RDONLY);
	
	struct termios tty;
	if(tcgetattr(fs,&tty) != 0){
		printf("error %i from tcgetarrt: %s\n", errno, strerror(errno));
	}

	tty.c_cc[VTIME] = 50;
	tty.c_cc[VMIN] = 0;
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~CRTSCTS;
	tty.c_cflag |= CREAD | CLOCAL;
	tty.c_cflag &= ~ICANON;
	tty.c_cflag &= ~ISIG;

	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);


	cfsetispeed(&tty, 2000000);

	tcsetattr(fs, TCSANOW, &tty);
	

	char read_buf [200];
	memset(read_buf,0,sizeof read_buf);
	printf("read from serial port\n");
	int len = 0;
	while(1){
		
		int n = read(fs, &read_buf[len], sizeof(read_buf)-len);
		if( n == 0)
		{
			printf("read time out\n");
			continue;
		}
		len = len + n;
		//int ret = findMagicBuf(read_buf, len);
		//if (ret > 0){
			//fclose(fd);
			//printf("close connections and exit\n");
			//return;
			//printf("close connections and exit\n");
			//return;
		//}
		if (len >= sizeof(read_buf))
		{
			printf("read 200 bytes\n");
			memset(read_buf,0,sizeof read_buf);
			len = 0;	
		}
	}

	int msgLen = findMagicBuf(fss);
	if(msgLen < 0)
	{
		printf("read error, close connections and exit\n");
	}
	printf("got msg len: %d close connections and exit\n");
		close(fss);
		return;

	
	
    //radioSerialPortReader(fd);
	radioSerialPortReader(fss);

	
	fclose(fd);
	printf("close connection bad msg cnt: %d good msg cnt %d\n", bad_msg_count, good_msg_count);
	*/
}

/*
int hex2int(bufptr)
{
	unsigned char valbuf [3];
	memcpy(valbuf, bufptr, 2);
	valbuf[3] = 0;
	int val = strtoul(valbuf, (char**)NULL, 16);
	//printf("hex2int val %d\n", val);
	return val;
}

int _fread(int fs, int len)
{
	int tocnt = 0;
	int rdlen = 0;
	while(rdlen < len){
		ssize_t n = read(fs, &readBuf[rdlen], len-rdlen);
		if(n < 0)
		{
			printf("error %i from tcgetarrt: %s\n", errno, strerror(errno));
			return -1;
		}
		if((int)n == 0)
		{
			printf("_fread time out cnt: %d\n", tocnt);	
			tocnt++;
			if(tocnt > 100)
				tocnt = 0;
		}
		rdlen = rdlen + (int)n;
		
	}
	printf("_fread got: %d bytes\n", rdlen);
	return rdlen;

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

		processRadioMessage(0,0, msgLen);
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
                processRadioMessage(0,i+6, i+6+msgLen);

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

int findMagicBuf(int sd)
{
	printf("enter findMagicBuf \n");
	//int ret = _fread(sd, 1);
	//if (ret < 0)
	//{
	//	return ret;
	//}
	int cnt = 0;
	while (1)
	{
		//int ret = _fread(sd, 100);
		//int ret = fread(sd, 100);
		int bts = fread(readBuf, 1, 100, sd);
		if(bts == 0)
		{
			printf("no stram data\n");
			sleep(1);
			continue;
		}
		//printf("read %d\n", buf[0]);
		//printf("match to %d\n", radioMagic[cnt]);
		for(int i = 0; i<100; i++)
		{
			if ((unsigned char)readBuf[i] == (unsigned char)radioMagic[cnt]){
        		cnt = cnt + 1;
				printf("found a match cnt: %d\n", cnt);
        		if (cnt >= 4){
					printf("found magic\n");
					_fread(sd, 2);
					int msgLen = (__uint16_t)(buf[0]) + ((__uint16_t)(buf[1])<<8) + 5;
            		return msgLen;
				}
			}
			else
			{
				//printf("found NO match\n");
				cnt = 0;
				continue;
			}
		}
	} 
	printf("failed to find magic\n"); 
	return -1;

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

void processRadioMessage(int pipe, int msg_len, unsigned char* msg) {
	// RSSI and message timestamp are prepended to the actual packet.

	// RSSI
	//unsigned char rssiRaw = (buf[st]);
	//rssiAdjusted := int16(rssiRaw) - 132 // -132 dBm, calculated minimum RSSI.
	//rssiDump978 := int16(1000 * (10 ^ (float64(rssiAdjusted) / 20)))
	//int rssiDump978 = (int)rssiRaw;

	//_ := uint32(msg[1]) + (uint32(msg[2]) << 8) + (uint32(msg[3]) << 16) + (uint32(msg[4]) << 24) // Timestamp. Currently unused.

	//msg = msg[5:]
    //int msglen = sizeof(msg)-5;
	//printf("enter processRadioMessage\n");
	char ret_buf [100];
    int msglen =  msg_len-5;

	int rs_errors = 0;

	printf("c code msglen %d \n", msglen);
	switch (msglen) 
    {
	case 552:
		//correct_uplink_frame(&buf[st+5], to, &rs_errors);
		correct_uplink_frame(&msg[5], to, &rs_errors);
        to[432]=0;
		//int cnt = sprintf((char*)toRelay, "+%s;ss=%d;", (char*)to, rssiDump978);
        //toRelay[cnt+1] = 0;
	case 48:
		//copy(to, msg);
        //memcpy(&to[0], &buf[st+5], msglen );
		printf("correct_adsb_frame \n");
		memcpy(&to[0], &msg[5], msglen );
		int i = correct_adsb_frame((unsigned char*)to, &rs_errors);
		printf("i return %d \n", i);
		if (i == 1) {
			// Short ADS-B frame.
            to[18]=0;

			good_msg_count += 1;

			struct uat_adsb_mdb mdb_zero; 
			
			uat_decode_adsb_mdb((char*)to, &mdb_zero);
			if(mdb_zero.callsign[0] == 0)
			{
				mdb_zero.callsign[0] = 'e';
				mdb_zero.callsign[1] = 'm';
				mdb_zero.callsign[2] = 'p';
				mdb_zero.callsign[3] = 't';
				mdb_zero.callsign[4] = 'y';
			}

			//uat_display_sv(&mdb_zero);

			/*printf("icao: %d callsign: %s lat: %f lon: %f alt: %d track: %d speed: %d v_speed %d\n", 
					mdb_zero.address, 
					mdb_zero.callsign ,
					mdb_zero.lat, 
					mdb_zero.lon,
					mdb_zero.altitude,
					mdb_zero.track,
					mdb_zero.speed,
					mdb_zero.vert_rate
					);*/

			/*int pb = sprintf(ret_buf, "icao: %d callsign: %s lat: %f lon: %f alt: %d track: %d speed: %d v_speed %d\n",
					mdb_zero.address, 
					mdb_zero.callsign ,
					mdb_zero.lat, 
					mdb_zero.lon,
					mdb_zero.altitude,
					mdb_zero.track,
					mdb_zero.speed,
					mdb_zero.vert_rate
				);*/

			printf("write to pipe\n");
			
			//fwrite(ret_buf, 1, pb, fss);
			//fflush(fss);

			send_result(pipe, &mdb_zero);

			//int cnt = sprintf((char*)toRelay, "-%s;ss=%d;", (char*)to, rssiDump978);
            //toRelay[cnt+1] = 0;
		} 
        else if (i == 2) {
			// Long ADS-B frame.
            to[34]=0;

			good_msg_count += 1;

			struct uat_adsb_mdb mdb_zero; 
			uat_decode_adsb_mdb((char*)to, &mdb_zero);
			if(mdb_zero.callsign[0] == 0)
			{
				mdb_zero.callsign[0] = 'e';
				mdb_zero.callsign[1] = 'm';
				mdb_zero.callsign[2] = 'p';
				mdb_zero.callsign[3] = 't';
				mdb_zero.callsign[4] = 'y';
			}

			//uat_display_sv(&mdb_zero);

			/*printf("icao: %d callsign: %s lat: %f lon: %f alt: %d track: %d speed: %d v_speed %d\n", 
					mdb_zero.address, 
					mdb_zero.callsign ,
					mdb_zero.lat, 
					mdb_zero.lon,
					mdb_zero.altitude,
					mdb_zero.track,
					mdb_zero.speed,
					mdb_zero.vert_rate
					);*/

			/*int pb = sprintf(ret_buf, "icao: %d callsign: %s lat: %f lon: %f alt: %d track: %d speed: %d v_speed %d\n",
					mdb_zero.address, 
					mdb_zero.callsign ,
					mdb_zero.lat, 
					mdb_zero.lon,
					mdb_zero.altitude,
					mdb_zero.track,
					mdb_zero.speed,
					mdb_zero.vert_rate
				);*/

			//printf("write to pipe\n");
			
			//fwrite(ret_buf, 1, pb, fss);
			//fflush(fss);
			
			send_result(pipe, &mdb_zero);

			//cnt= sprintf((char*)toRelay, "-%s;ss=%d;",  (char*)to, rssiDump978);
            //toRelay[cnt+1] = 0;
		}
		else{
			char cb[20];
			printf("fec fail\n");
			strcpy(cb, "fec fail\n");                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
			int nbytes = strlen("fec fail\n");
			//int wc = fwrite(cb, 1, nbytes, fss);
			int wc = write(pipe, cb, nbytes);
			printf("write cnt byts: %d\n", wc);
			//fflush(fss);
			printf("wrote fec fail to pipe\n");
			
			bad_msg_count += 1;
		}
		//printf("return from processRadioMessage\n");

	//default:
		//print("processRadioMessage(): unhandled message size \n", msglen);
	}

	//if (strlen((char*)toRelay) > 0 && rs_errors != 9999) {
	//if (0) {
		//unsigned char tb[50];
		//memset(tb,0,sizeof tb);

		//int len = 0;
		//len = strlen(toRelay);
		//for(int i = 0; i < len; i++)
		//{
			//printf("%d ", toRelay[i]);
		//}
		//printf("\n");
		//fflush(stdout);
		//sprintf(tb, "%s", toRelay[0]);

		//printf("frame: %s\n", toRelay);
		//int i = 0;
		//fflush(stdout);
	//}
}

void send_result(int pipe, struct uat_adsb_mdb* mdb_zero)
{
	char ret_buf [800];
	int pb = sprintf(ret_buf, "icao %d :callsign %s :lat %f :lon %f :adsb_altitude %d :adsb_heading %d :hor_velocity %d :ver_velocity %d\n",
					mdb_zero->address, 
					mdb_zero->callsign ,
					mdb_zero->lat, 
					mdb_zero->lon,
					mdb_zero->altitude,
					mdb_zero->track,
					mdb_zero->speed,
					mdb_zero->vert_rate
				);

	printf("write to pipe\n");
	//fwrite(ret_buf, 1, pb, fss);
	//fflush(fss);
	write(pipe, ret_buf, pb);

}

// gib - display adsb from aircraft
void uat_display_sv(const struct uat_adsb_mdb *mdb)
{
    if (!mdb->has_sv)
        return;

    printf(
            "SV:\n"
            " NIC:               %u\n",
            mdb->nic);

    if (mdb->position_valid)
        printf(
                " Latitude:          %+.4f\n"
                " Longitude:         %+.4f\n",
                mdb->lat,
                mdb->lon);

    switch (mdb->altitude_type) {
    case ALT_BARO:
        printf(
                " Altitude:          %d ft (barometric)\n",
                mdb->altitude);
        break;
    case ALT_GEO:
        printf(
                " Altitude:          %d ft (geometric)\n",
                mdb->altitude);
        break;
    default:
        break;
    }

    if (mdb->ns_vel_valid)
        printf(
                " N/S velocity:      %d kt\n",
                mdb->ns_vel);

    if (mdb->ew_vel_valid)
        printf(
                " E/W velocity:      %d kt\n",
                mdb->ew_vel);

    switch (mdb->track_type) {
    case TT_TRACK:
        printf(
                " Track:             %u\n",
                mdb->track);
        break;
    case TT_MAG_HEADING:
        printf(
                " Heading:           %u (magnetic)\n",
                mdb->track);
        break;
    case TT_TRUE_HEADING:
        printf(
                " Heading:           %u (true)\n",
                mdb->track);
        break;
    default:
        break;
    }
}