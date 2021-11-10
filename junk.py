import os
import subprocess
import sys, tty


"""p = subprocess.Popen([sys.executable, "get_key.py"], 
    stdout = subprocess.PIPE, stdin = sys.stdin)
    
while True:
    _, out = p.communicate()
    print("pipe key ", out)"""

os.system("xterm -geometry 225x75+20+20 -e 'python3 /home/pi/PhidgetInsurments/pix_hawk_compass.py'")
#os.system("xterm -e 'exit()'")