"""
Circular Trajectory Tracking

"""

#-----IMPORT
from __future__ import print_function
import socket
import time
import math
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


#--Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):

	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,
		0,0,
		mavutil.mavlink.MAV_FRAME_BODY_NED,
		0b0000111111000111,#--BITMASK -> Consider only the velocities
		0,0,0,             #--POSITION
		vx,vy,vz,          #--VELOCITY
		0,0,0,			   #--ACCELERATIONS
		0,0)
	vehicle.send_mavlink(msg)
	vehicle.flush()

def clear_mission(vehicle):

	cmds = vehicle.commands
	vehicle.commands.clear()
	vehicle.flush()

	#After clearing the mission you must re-download the mission from the vehicle
	#before vehicle.commands can be used again
	#
	cmds = vehicle.commands
	cmds.download()
	cmds.wait_ready()

def download_mission(vehicle):
	
	cmds = vehicle.commands
	cmds.download()
	cmds.wait_ready()

def get_current_mission(vehicle):

	print("Downloading mission")
	download_mission(vehicle)
	missionList = []
	n_WP        =  0
	for wp in vehicle.commands:
		missionList.append(wp)
		n_WP +=1

	return n_WP, missionList

def ChangeMode(vehicle,mode):
	while vehicle.mode != VehicleMode(mode):
		vehicle.mode = VehicleMode(mode)
		time.sleep(0.5)

	return True

#----NEW Functions 

def get_distance_meters(aLocation1,aLocation2):
	"""
	Returns an approximate distance in meters between two Location objects
	This is an approximate method that works far from Earth's poles
	"""
	dlat = aLocation2.lat - aLocation1.lat
	dlon = aLocation2.lon - aLocation1.lon

	return math.sqrt((dlat*dlat) + (dlon*dlon))*1.113195e5

def distance_to_current_waypoint(vehicle):

	""" 
	Returns the distance to the current waypoint

	"""

	nextwaypoint = vehicle.commands.next

	if nextwaypoint == 0:
		nextwaypoint += 2

	missionitem = vehicle.commands[nextwaypoint-1] 
	lat = missionitem.x
	lon = missionitem.y
	alt = missionitem.z

	target_waypoint = LocationGlobalRelative(lat,lon,alt)
	distance_to_point = get_distance_meters(vehicle.location.global_frame, target_waypoint)
	return distance_to_point

"""
def get_bearing(my_locaiton, tat_lcation):

	dlat = tat_lcation.lat - my_locaiton.lat 
 	dlon = tat_lcation.lon - my_locaiton.lon

 	return math.atan2(dlon,dlat)
 	"""

def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.

    This method is an approximation, and may not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """	
    dlon = aLocation2.lon - aLocation1.lon
    dlat = aLocation2.lat - aLocation1.lat
    bearing = math.atan2(dlon, dlat)
    return bearing;

def bearing_to_current_waypoint(vehicle):
 	

 	nextwaypoint = vehicle.commands.next
	
	if nextwaypoint == 0:
 		nextwaypoint += 2

 	missionitem = vehicle.commands[nextwaypoint-1]
 	lat = missionitem.x
 	lon = missionitem.y
 	alt = missionitem.z

 	target_waypoint = LocationGlobalRelative(lat,lon,alt)

 	bearing = get_bearing(vehicle.location.global_frame,target_waypoint)

 	return bearing

def condition_yaw(heading,relative=False):

 	msg = vehicle.message_factory.command_long_encode(
 		0,0,			# target system and target component
 		mavutil.mavlink.MAV_CMD_CONDITION_YAW,  #command type
 		0, 				#
 		heading,		# param 1: yaw in degress in [0,360]
 		0,				# param 2: yaw speed deg/s
 		1,				# param 3: direction -1 ccw, 1 cw
 		relative,		# param 4: relative offset
 		0,0,0)         	# params not used

 	vehicle.send_mavlink(msg)

def saturate(value, minimum, maximum):
 	if value > maximum: value = maximum
 	if value < minimum: value = minimum
 	return value


def add_angles(ang1,ang2):

 	ang= ang1+ang2

 	if ang > 2.0*math.pi: 
 		ang = ang - 2.0*math.pi
 	if ang < 0.0: 
 		ang = ang + 2.0*math.pi

 	return ang

def UDPbroadcast(vehicle):
	global b_msg
	ss = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	ss.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
 	PORT = 1060
 	network = '<broadcast>'
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
	ss.sendto(str(b_msg).encode('utf-8'), (network, PORT))

def UDPlistener():
	global s
	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
	PORT = 1060
	s.bind(('', PORT))
	print('Listening for broadcast at ', s.getsockname())
	



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
Swarm_No = 4
#Swarm_vehicle = numpy.zeros((Swarm_No,len(b_msg)))
Swarm_vehicle = [[0 for i in range(len(b_msg))] for j in range(Swarm_No)]

 #----------------------------------------------------------
 #--------------CONNECTION-------------------------------
 #-------Connect to the vehicle

print("Connecting.....")
vehicle = connect('udp:127.0.0.1:14551')
#vehicle = connect('/dev/tty0')
UDPlistener()
time.sleep(1)
arm_and_takeoff(10)
 #----------------------------------------------------------
 #--------------MAIN FUNTION
 #----------------------------------------------------------
while True:

	UDPbroadcast(vehicle)

	data, address = s.recvfrom(65535)
	#print('Server received from {}:{}'.format(address, data.decode('utf-8')))
	datadecode = data[1:len(data)-1].split(',')
	vehicleID = int(datadecode[0]) 
	if vehicleID != vehicle._heartbeat_system :
		Swarm_vehicle[vehicleID-1][0] = int(datadecode[0])
		for k in range(1,len(datadecode)):
			Swarm_vehicle[vehicleID-1][k] = float(datadecode[k])

	


	#----calculate the time of flight
	time_flight = 2.0*math.pi*radius/gnd_speed*n_turns
	time0 = time.time()

	mode = 'MISSIOM'
	print(">> Switch to MISSIOM")


	if time.time() > time0 + time_flight:
		ChangeMode(vehicle, 'RTL')
		mode = 'BACK'
		print(">> Time to go home")




 	time.sleep(0.5)
