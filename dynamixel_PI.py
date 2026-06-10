import matplotlib.pyplot as plt
import time
from dynamixel_sdk import *


portHandler = PortHandler("/dev/ttyUSB0")
packetHandler = PacketHandler(2.0)
start_time= time.time()

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
packetHandler.write1ByteTxRx(portHandler, dxl_id, torque_on_address, 0)
packetHandler.write1ByteTxRx(portHandler, dxl_id, 11, 3)  # position mode
packetHandler.write2ByteTxRx(portHandler, dxl_id, 588, 462)  #  P 
packetHandler.write2ByteTxRx(portHandler, dxl_id, 586, 16)   # I 
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, torque_on_address, data)

print("comm =", dxl_comm_result)
print("error =", dxl_error)

time_log = []
goal_log = []
pos_log = []

start_time = time.time()

if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")

while True:
    user_input=input("Enter target position (-103846 to 103846 or q to quit): ")
    if user_input.lower()=='q':
       break
    try:
        target_position = int(user_input)
    except ValueError:
        print("Please enter an integer.")
        continue
    
    if target_position < - 103846 or target_position > + 103846:
        print("Position must be between - 103846 and + 103846.")
        continue
    
    target_position_u32=target_position & 0xFFFFFFFF 

    goal_position_address = 596
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_id, goal_position_address, target_position_u32)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        
    loop_start = time.time()

    while True:
        present_position_address = 611
        present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, present_position_address)
        if present_position > 0x7FFFFFFF:
            present_position -= 0x100000000
            
        t = time.time() - start_time
            
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            
        time_log.append(t)
        goal_log.append(target_position)
        pos_log.append(present_position)
        
        if dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print(f"Current Position: {present_position}")
        
        if abs(target_position - present_position) <= 200:
            break
        
        if time.time() - loop_start > 5:
         print("timeout reached")
         break
        time.sleep(0.01)
data = 0
packetHandler.write1ByteTxRx(portHandler, dxl_id, torque_on_address, data)
portHandler.closePort()

plt.plot(time_log, goal_log, label="Goal")
plt.plot(time_log, pos_log, label="Position")

plt.xlabel("Time (s)")
plt.ylabel("Position")
plt.legend()
plt.grid()

plt.savefig("plot.png", dpi=200)
plt.show()