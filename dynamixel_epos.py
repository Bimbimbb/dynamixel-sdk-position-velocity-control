import matplotlib.pyplot as plt
import time
from dynamixel_sdk import *
import csv

portHandler = PortHandler("/dev/ttyUSB0")
packetHandler = PacketHandler(2.0)

if portHandler.openPort():
  print("Succeeded to open the port!")
else:
  print("Failed to open the port!")
  exit()

if portHandler.setBaudRate(57600):
  print("Succeeded to change the baudrate!")
else:
  print("Failed to change the baudrate!")
  exit()

model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, 1)

print("Model:", model_number)
print("Comm result:", dxl_comm_result)
print("Error:", dxl_error)

if dxl_comm_result == COMM_SUCCESS:
    print("Ping successful!")
else:
    print(packetHandler.getTxRxResult(dxl_comm_result))
    
dxl_id = 1
torque_on_address = 562
data = 1
pulses_per_rev = 207692
packetHandler.write1ByteTxRx(portHandler, dxl_id, torque_on_address, 0)

# set velocity and acceleration profile
packetHandler.write1ByteTxRx(portHandler, dxl_id, 11, 4)  # position mode
packetHandler.write4ByteTxRx(portHandler, dxl_id, 600, 2500)  # Goal Velocity
packetHandler.write4ByteTxRx(portHandler, dxl_id, 606, 1)   # Goal Acceleration
packetHandler.write2ByteTxRx(portHandler, dxl_id, 588, 462)  #  P 
packetHandler.write2ByteTxRx(portHandler, dxl_id, 586, 16)   # I 
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, torque_on_address, data)

print("comm =", dxl_comm_result)
print("error =", dxl_error)

time_log = []
goal_log = []
pos_log = []
vel_log = []

start_time = time.time()

if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")
    
with open('data_pos.csv','w', newline = '') as f:
    writer = csv.writer(f)
    writer.writerow(['time','goal_position','present_position','present_velocity'])
    
    while True:
       user_input = input("Enter revolutions and final angle (or q to quit): ")
       if user_input.lower()=='q':
           break
       try:
            revolution,angle = map(int,user_input.split())
            target_position = revolution * pulses_per_rev + int(angle * pulses_per_rev/360)
       except ValueError:
           print("Please enter an integer.")
           continue
    
       if target_position < - 2147483648 or target_position > + 2147483647:
           print("Position is out of the extended range.")
           continue
    
       target_position_u32=target_position & 0xFFFFFFFF 

       goal_position_address = 596
       dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_id, goal_position_address, target_position_u32)
       
       if dxl_error != 0:
             print("%s" % packetHandler.getRxPacketError(dxl_error))
        
       loop_start = time.time()
       start_time = time.time()
       
       while True:
          present_position_address = 611
          present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, present_position_address)
          present_velocity, vel_comm_result, vel_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, 615)
          if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            continue 
          
          if present_position > 0x7FFFFFFF:
              present_position -= 0x100000000
              
          if present_velocity > 0x7FFFFFFF:
              present_velocity -= 0x100000000
            
          t = time.time() - start_time
          if time_log and (t < time_log[-1] or t > time_log[-1]+1.0):
             t = time_log[-1] + 0.01  # use expected time s
            
          time_log.append(t)
          goal_log.append(target_position)
          pos_log.append(present_position)
          vel_log.append(present_velocity)
            
          writer.writerow([t,target_position,present_position,present_velocity])
        
          if dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
          print(f"Current Position: {present_position}")
        
          if abs(target_position - present_position) <= 500 and abs(present_velocity) < 100:
            break
        
          if time.time() - loop_start > 200:
            print("timeout reached")
            print("timeout check:", time.time() - loop_start)
            break
          time.sleep(0.005)


data = 0
packetHandler.write1ByteTxRx(portHandler, dxl_id, torque_on_address, data)
portHandler.closePort()

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
ax1.plot(time_log, goal_log, label="Goal")
ax1.plot(time_log, pos_log, label="Position")
ax1.set_ylabel("Position")
ax1.legend()
ax1.grid()

ax2.plot(time_log, vel_log, label="Velocity", color='green')
ax2.set_ylabel("Velocity")
ax2.set_xlabel("Time (s)")
ax2.legend()
ax2.grid()

filename = f"plot_{int(time.time())}.png"
plt.savefig(filename, dpi=200)
print("Saved:", filename)