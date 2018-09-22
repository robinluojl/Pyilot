"""
Circular Trajectory Tracking

"""

#-----IMPORT
from __future__ import print_function
import socket
import time
import math
import threading
import numpy
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

#----------------------------------------------
#----------------------Functions
#----------------------------------------------
#-- Define arm and takeoff
def arm_and_takeoff(altitude):

	while not vehicle.is_armable:
		print("waiting to be armable")
		time.sleep(1)

	print("Arming motors")
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True

	while not vehicle.armed:
		time.sleep(1)

	print("Taking Off")
	vehicle.simple_takeoff(altitude)

	while True:
		v_alt = vehicle.location.global_relative_frame.alt
		print(">> Altitude = %.lf m"%v_alt)
		if v_alt >= altitude-1.0:
			print("Target altitude reached")
			break
		time.sleep(1)

def ChangeMode(vehicle,mode):
	while vehicle.mode != VehicleMode(mode):
		vehicle.mode = VehicleMode(mode)
		time.sleep(0.5)

	return True


def UDPbroadcast(vehicle,s,b_msg):

 	b_msg = [vehicle._heartbeat_system,
 			time.time(),
     		vehicle.location._lat,
     		vehicle.location._lon,
     		vehicle.location._alt,
     		vehicle.location._relative_alt,
     		vehicle._vx,
     		vehicle._vy,
     		vehicle._vz,
     		vehicle._pitch,
     		vehicle._roll,
     		vehicle._yaw,
        	vehicle._heading,
        	vehicle._airspeed,
        	vehicle._groundspeed ]
	s.sendto(str(b_msg).encode('utf-8'), (network, PORT))


def UDPlistener(vehicle,s,Swarm_vehicle,cmd,broadcast_rate):
	try:
	    while True:
	    	current = time.time()
            elapsed = 0.0
            UDPbroadcast(vehicle,s,b_msg)
            data, address = s.recvfrom(1024)
			#print('Server received from {}:{}'.format(address, data.decode('utf-8')))
            datadecode = data[1:len(data)-1].split(',')
            vehicleID = int(datadecode[0])
			#print(address)
			#print(datadecode)
            if vehicleID == 65535:
				print('get conmmand')
				if datadecode[1] == 0.0 :
					cmd = 'Takeoff'
					print('Takeoff')
				if datadecode[1] == 1.0 :
					cmd = 'Swarm'
					print('Swarm')
					for k in range(1,len(datadecode)):
						Swarm_vehicle[vehicle._heartbeat_system-1][k] = float(datadecode[k])
				if datadecode[1] == 2.0 :
					cmd = 'Mission'
					print('Mission')
				if datadecode[1] == 3.0 :
					cmd = 'Stop'
					print('Stop')
				if datadecode[1] == 4.0 :
					cmd = 'RTL'
					print('RTL')

            elif vehicleID != vehicle._heartbeat_system :
				print(address)
				print(datadecode)
				Swarm_vehicle[vehicleID-1][0] = int(datadecode[0])
				for k in range(1,len(datadecode)):
					Swarm_vehicle[vehicleID-1][k] = float(datadecode[k])

            while elapsed < broadcast_rate:
          		elapsed = time.time() - current
	except Exception,error:
		print("Error on UDPlistener thread: "+str(error))
		UDPlistener(vehicle,s,Swarm_vehicle,broadcast_rate)

def OffboardCtrl(vehicle,Swarm_vehicle,cmd,mode,control_rate):
	while True:
		time0 = time.time()
		dt = 0

		if cmd == 'Takeoff':
		   mode = 'TAKEOFF'
		   arm_and_takeoff(10)
		   mode = 'AIR'
		   print('>> Takeoff to 10m')

		if ((cmd == 'Swarm') & (mode == 'AIR')):
			print('>> Switch to Swarm')

		if ((cmd == 'Mission') & (mode == 'AIR')):

			mode = 'MISSIOM'
			ChangeMode(vehicle,'auto')
			print(">> Switch to MISSIOM")


		if cmd == 'RTL':
			ChangeMode(vehicle, 'RTL')
			mode = 'BACK'
			print(">> Time to go home")

		while dt < control_rate:
			dt = time.time() - time0

def Swarm(Vehicle,Swarm_vehicle):
	print('Swarm offboard control')


 #--------------------INITIALIZE--------------------------
gnd_speed 			= 8                	# [m/s]
radius			= 80               	# [m]
max_lat_speed		= 4                  # [m/s]
k_err_vel			= 0.2	# [1/s]
n_turns			= 3
direction			= 1		# 1 cw, -1 ccw
b_msg = [1.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0]
mode 				= 'GROUND'
cmd                 ='Null'
Swarm_No = 4
#Swarm_vehicle = numpy.zeros((Swarm_No,len(b_msg)))
Swarm_vehicle = [[0 for i in range(len(b_msg))] for j in range(Swarm_No)]

broadcast_rate = 0.01 # 100 hz loop cycle
control_rate = 0.1 # 10 hz loop cycle

#------------------------------------------------------------
#-----------Define UDP socket
#------------------------------------------------------------
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
PORT = 1060
network = '<broadcast>'
s.bind(('', PORT))
print('Listening for broadcast at ', s.getsockname())
time.sleep(1)

#----------------------------------------------------------
#--------------CONNECTION-------------------------------
#-------Connect to the vehicle

print("Connecting Vehicle.....")
vehicle = connect('udp:127.0.0.1:14561')
#vehicle = connect('/dev/tty0')

#-----------------------------------------------------------------------
#--------------Define threads
#-----------------------------------------------------------------------
vehicleThreads = []
T1 = threading.Thread(target=UDPlistener,args=(vehicle,s,Swarm_vehicle,cmd,broadcast_rate))
vehicleThreads.append(T1)
T2 = threading.Thread(target=OffboardCtrl,args=(vehicle,Swarm_vehicle,cmd,mode,control_rate))
vehicleThreads.append(T2)


 #----------------------------------------------------------
 #--------------MAIN FUNTION
 #----------------------------------------------------------


if __name__ == '__main__':

    try:
        for t in vehicleThreads:
			t.setDaemon(True)
			t.start()
    except Exception,error:
        print( "Error on main script thread: "+str(error))
        vehicle.close()



