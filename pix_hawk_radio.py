from ast import Pass
from re import I
from sys import breakpointhook
from typing import SupportsInt
from numpy import dtype
import serial
import time
from threading import Thread
import numpy as np
import ctypes
import sys
from pix_hawk_util import KeyBoard
import traceback

radioMagic = [0x0a, 0xb0, 0xcd, 0xe0]

class RadioThread():
    def __init__(self, conn_str):
        self.conn_str = conn_str
        self.ser = None
        #self.read_td = Thread(target=self.target)
        self.read_td = Thread(target=self.radioSerialPortReader)
        self.fec_lib = ctypes.CDLL('./fec_lib.so')
        self.run_thread = True
        self.maxSignalStrength = 0
        self.key_board = KeyBoard.get_instance()
        
    
    def connect(self):
        self.ser = serial.Serial(self.conn_str, baudrate=2000000, timeout=5)
        return self.ser != None

    def processRadioMessage(self, msg):
        rssiRaw = msg[0]
        rssiDump978 = rssiRaw
        msg = msg[5:]
        toRelay = None
        rs_errors = 0
        if len(msg) == 552:
            # correct_uplink xfers data from msg buf, to tobuf via pointer to each
            # and returns result in rs_errors
            #C.correct_uplink_frame((*C.uint8_t)(unsafe.Pointer(&msg[0])), (*C.uint8_t)(unsafe.Pointer(&to[0])), (*C.int)(unsafe.Pointer(&rs_errors)))
            to = bytearray(432)
            msg_cnt = len(msg)

            #int correct_uplink_frame(uint8_t *from, uint8_t *to, int *rs_errors)
            self.fec_lib.correct_uplink_frame.argtypes = \
                ctypes.POINTER(ctypes.c_byte), ctypes.POINTER(ctypes.c_byte), ctypes.POINTER(ctypes.c_int)

            array_type_msg = ctypes.c_byte * msg_cnt
            array_type_to = ctypes.c_byte * 432

            rs_errors = ctypes.c_int32()

            # reed solomen from for uplink is (92,72) * 6 blocks = 984
            # we expect 552

            self.fec_lib.correct_uplink_frame(array_type_msg.from_buffer(msg), array_type_to.from_buffer(bytes(to)), \
                    ctypes.byref(rs_errors))

            #toRelay = Sprintf("+%s;ss=%d;", hex.EncodeToString(to[:432]), rssiDump978)

            rs = str(to[:432])
            toRelay = '+{0}{1)'.format(rs, rssiDump978)

        elif len(msg) == 48:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
            #to = bytearray(48)
		    #copy(to, msg)
            to = msg[:48]
            msg_cnt = len(msg)

            #int correct_adsb_frame(uint8_t *to, int *rs_errors)

            
            array_type_to = ctypes.c_byte * 48
            rs_errors = ctypes.c_int32()

            # reed solomen from for adsb msg is: (30,18) = 48  or (48,34)  =  88   
            #  fec c code tries long then short

            i = 0
            try:
                i = int(self.fec_lib.correct_adsb_frame(array_type_to.from_buffer(bytes(to)), ctypes.byref(rs_errors)))
            except:
                traceback.print_exc()


            if i == 1:
			    #Short ADS-B frame.
                rs = str(to[:18])
                toRelay = '-{0}{1)'.format(rs, rssiDump978)
                #toRelay = fmt.Sprintf("-%s;ss=%d;", hex.EncodeToString(to[:18]), rssiDump978)
            elif i == 2:
			    #Long ADS-B frame.
                #toRelay = fmt.Sprintf("-%s;ss=%d;", hex.EncodeToString(to[:34]), rssiDump978)
                rs = str(to[:34])
                toRelay = '-{0}{1)'.format(rs, rssiDump978)
        else:
             print ("processRadioMessage(): unhandled message size {0}", len(msg))

        if toRelay != None:
            sys.stdout.write(toRelay)  # gib - can I send this to uat_decode.c in c modules??

        #if len(toRelay) > 0 and rs_errors != 9999:
        #    o, msgtype= self.parseInput(toRelay)
            
        
        #if o != None and msgtype != 0:
			#relayMessage(msgtype, o)
            #sys.stdout.write(o)
		
    def parseInput(self, buf):  # gib - same as uat_decode.c ???
        x = buf.split(';') # Discard everything after the first ';'.
        s = x[0]
        if len(s) == 0:
            return None, 0

        msgtype = 0
        msgtype = msgtype.astype(np.uint16)
        isUplink = False
        if s[0] == '+':
            isUplink = True

        thisSignalStrength = 0
        if len(x) >= 3:
		    # See if we can parse out the signal strength.
            ss = x[2]
            #if strings.HasPrefix(ss, "ss="): 
            ssInt = None
            if ss[0:2] == 'ss':
                ssStr = ss[3:]
                try:
                    ssInt = int(ssStr)
                except:
                    pass

            if ssInt != None:
                if isUplink and (ssInt > self.maxSignalStrength): # only look at uplinks; ignore ADS-B and TIS-B/ADS-R messages
                    self.maxSignalStrength = ssInt
	
	
        a = 1
        b = 2
        return a, b
    """   
    func parseInput(buf string) ([]byte, uint16) {
	//FIXME: We're ignoring all invalid format UAT messages (not sending to datalog).
	x := strings.Split(buf, ";") // Discard everything after the first ';'.
	s := x[0]
	if len(s) == 0 {
		return nil, 0
	}
	msgtype := uint16(0)
	isUplink := false

	if s[0] == '+' {
		isUplink = true
	}

	var thisSignalStrength int

	if /*isUplink &&*/ len(x) >= 3 {
		// See if we can parse out the signal strength.
		ss := x[2]
		//log.Printf("x[2] = %s\n",ss)
		if strings.HasPrefix(ss, "ss=") {
			ssStr := ss[3:]
			if ssInt, err := strconv.Atoi(ssStr); err == nil {
				thisSignalStrength = ssInt
				if isUplink && (ssInt > maxSignalStrength) { // only look at uplinks; ignore ADS-B and TIS-B/ADS-R messages
					maxSignalStrength = ssInt
				}
			} else {
				//log.Printf("Error was %s\n",err.Error())
			}
		}
	}

	if s[0] == '-' {
		parseDownlinkReport(s, int(thisSignalStrength))
	}

	s = s[1:]
	msglen := len(s) / 2

	if len(s)%2 != 0 { // Bad format.
		return nil, 0
	}

	if isUplink && msglen == UPLINK_FRAME_DATA_BYTES {
		msgtype = MSGTYPE_UPLINK
	} else if msglen == 48 {
		// With Reed Solomon appended
		msgtype = MSGTYPE_LONG_REPORT
	} else if msglen == 34 {
		msgtype = MSGTYPE_LONG_REPORT
	} else if msglen == 18 {
		msgtype = MSGTYPE_BASIC_REPORT
	} else {
		msgtype = 0
	}

	if msgtype == 0 {
		log.Printf("UNKNOWN MESSAGE TYPE: %s - msglen=%d\n", s, msglen)
	}

	// Now, begin converting the string into a byte array.
	frame := make([]byte, UPLINK_FRAME_DATA_BYTES)
	hex.Decode(frame, []byte(s))

	var thisMsg msg
	thisMsg.MessageClass = MSGCLASS_UAT
	thisMsg.TimeReceived = stratuxClock.Time
	thisMsg.Data = buf
	thisMsg.Signal_amplitude = thisSignalStrength
	if thisSignalStrength > 0 {
		thisMsg.Signal_strength = 20 * math.Log10((float64(thisSignalStrength))/1000)
	} else {
		thisMsg.Signal_strength = -999
	}
	thisMsg.Products = make([]uint32, 0)
	if msgtype == MSGTYPE_UPLINK {
		// Parse the UAT message.
		uatMsg, err := uatparse.New(buf)
		if err == nil {
			uatMsg.DecodeUplink()
			towerid := fmt.Sprintf("(%f,%f)", uatMsg.Lat, uatMsg.Lon)
			thisMsg.ADSBTowerID = towerid
			// Get all of the "product ids".
			for _, f := range uatMsg.Frames {
				thisMsg.Products = append(thisMsg.Products, f.Product_id)
				UpdateUATStats(f.Product_id)
				weatherRawUpdate.SendJSON(f)
			}
			// Get all of the text reports.
			textReports, _ := uatMsg.GetTextReports()
			for _, r := range textReports {
				registerADSBTextMessageReceived(r, uatMsg)
			}
			thisMsg.uatMsg = uatMsg
		}
	}

	MsgLog = append(MsgLog, thisMsg)
	logMsg(thisMsg)

	return frame, msgtype
}"""

        
    def target(self):
        try:
            if not self.read_td.is_alive():
                self.read_td.start()
                print('radio thread started')

            file = open('/home/pi/PhidgetInsurments/mag_dataadsb_log.txt', 'ab')
            buf = []
            bufcnt = 0

            
            cnt = 0
            while self.run_thread:

                key = self.key_board.get_key()
                if key == '\n':
                    file.close()
                    self.run_thread = False
                
                ln = self.ser.read(1)
                if len(ln) == 0:
                    print('timeout')
                    continue
    
                byte = ord(ln)
                if byte == radioMagic[cnt]:
                    cnt += 1
                    if cnt >= 3:
                        cnt = 0
                    print('got magic')
                else:
                    cnt = 0
           
                buf.append(byte)
                bufcnt += 1
                if bufcnt > 10:
                    file.write(bytes(buf))
                    bufcnt = 0

                print(byte)

            print('radio  thread stopped')
            KeyBoard.stop()

        except:
            file.close()
            self.run_thread = False
            traceback.print_exc()


    def radioSerialPortReader(self):

        #if not self.read_td.is_alive():
        #    self.read_td.start()
        #    print('radio thread started')
        file = open('/home/pi/PhidgetInsurments/mag_dataadsb_log.txt', 'rb')

        buf = []
        while self.run_thread:
           
            #ln = self.ser.read(1)
            #if len(ln) == 0:
                #print('timeout')
                #continue

            #byte = file.read(1)
            buf = file.read(100)

            

            #byte = ord(ln)
            #byte = ord(byte)
            #buf.append(byte)
            bufLen = len(buf)
            if bufLen < 6:
                continue

            finBuf = []
            numMessages = 0
            for i in range(0,bufLen-6):
                if buf[i] == radioMagic[0] and buf[i+1] == radioMagic[1] and buf[i+2] == radioMagic[2] and buf[i+3] == radioMagic[3]:
                    
                    msgLen = int(buf[i+4] + (buf[i+5]<<8) + 5) # 5 bytes for RSSI and TS.
                    
                    if bufLen < i+6+msgLen:
                        break # Wait for more of the message to come in.
				
                    #Message is long enough.
                    self.processRadioMessage(buf[i+6 : i+6+msgLen])
				    #Remove everything in the buffer before this message.
                    finBuf = buf[i+6+msgLen:]
                    numMessages += 1
            if numMessages > 0:
                buf = finBuf

        print('radio thread stoped')
        KeyBoard.stop()

    """tmpBuf := make([]byte, 1024) // Read buffer.
	var buf []byte               // Message buffer.
	for {
		n, err := serialPort.Read(tmpBuf)
		if err != nil {
			log.Printf("serial port err, shutting down radio: %s\n", err.Error())
			return
		}
		buf = append(buf, tmpBuf[:n]...)
		bufLen := len(buf)
		var finBuf []byte   // Truncated buffer, with processed messages extracted.
		var numMessages int // Number of messages extracted.
		// Search for a suitable message to extract.
		for i := 0; i < bufLen-6; i++ {
			if (buf[i] == radioMagic[0]) && (buf[i+1] == radioMagic[1]) && (buf[i+2] == radioMagic[2]) && (buf[i+3] == radioMagic[3]) {
				// Found the magic sequence. Get the length.
				msgLen := int(uint16(buf[i+4])+(uint16(buf[i+5])<<8)) + 5 // 5 bytes for RSSI and TS.
				// Check if we've read enough to finish this message.
				if bufLen < i+6+msgLen {
					break // Wait for more of the message to come in.
				}
				// Message is long enough.
				processRadioMessage(buf[i+6 : i+6+msgLen])
				// Remove everything in the buffer before this message.
				finBuf = buf[i+6+msgLen:]
				numMessages++
			}
		}
		if numMessages > 0 {
			buf = finBuf
		}
	}
}"""


if __name__ == '__main__':

    rt = RadioThread('/dev/serial/by-id/usb-Stratux_Stratux_UATRadio_v1.0_DO0271Z9-if00-port0')
    if rt.connect():
        rt.read_td.start()
        time.sleep(500)


"""ser = serial.Serial('/dev/serial/by-id/usb-Stratux_Stratux_UATRadio_v1.0_DO0271Z9-if00-port0', baudrate=2000000, timeout=5)
#ser.open()
cnt = 0
while True:
    ln = ser.read(1)
    if len(ln) == 0:
        print('timeout')
        continue
    #byte = ln.decode("utf-8")
    #byte = ln.decode('hex')
    byte = ord(ln)
    if byte == magic[cnt]:
        cnt += 1
        if cnt >= 3:
            cnt = 0
            print('got magic')
    else:
        cnt = 0
    print(byte)::"""