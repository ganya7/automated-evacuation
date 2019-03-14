import serial
import time
arduino = serial.Serial("COM3",9600)
# arduino.baudrate=9600
danger_nodes = set()
for i in range(1,100):
	data = arduino.readline()
	data = data.decode('utf-8')
	no = "888"
	if no in data:
		danger_nodes.add(int(no))
	else:
		print(data)
	print("danger: ",danger_nodes)
	time.sleep(0.5)