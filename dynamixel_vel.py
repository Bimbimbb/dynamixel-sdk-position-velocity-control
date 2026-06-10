import matplotlib.pyplot as plt
import time
from dynamixel_sdk import *

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
if dxl_comm_result == COMM_SUCCESS:
    print("Ping successful!")
else:
    print(packetHandler.getTxRxResult(dxl_comm_result))

dxl_id = 1
torque_on_address = 562

# Disable torque before changing mode
packetHandler.write1ByteTxRx(portHandler, dxl_id, torque_on_address, 0)

# Set velocity mode
packetHandler.write1ByteTxRx(portHandler, dxl_id, 11, 1) # operation mode
packetHandler.write2ByteTxRx(portHandler, dxl_id, 588, 373)  #  P 
packetHandler.write2ByteTxRx(portHandler, dxl_id, 586, 20)   # I 

# Enable torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, torque_on_address, 1)
if dxl_comm_result != COMM_SUCCESS:
    print(packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print(packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")

time_log = []
goal_log = []
vel_log = []

start_time = time.time()

while True:
    user_input = input("Enter target velocity (-8000 to 8000, or q to quit): ")
    if user_input.lower() == 'q':
        break
    try:
        target_velocity = int(user_input)
    except ValueError:
        print("Please enter an integer.")
        continue

    if target_velocity < -8000 or target_velocity > 8000:
        print("Velocity must be between -8000 and 8000.")
        continue

    target_velocity_u32 = target_velocity & 0xFFFFFFFF
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
        portHandler, dxl_id, 600, target_velocity_u32
    )
    if dxl_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print(packetHandler.getRxPacketError(dxl_error))

    # Log for 3 seconds
    t_start = time.time()
    present_position_address = 611
    present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, present_position_address)
    if present_position > 0x7FFFFFFF:
         present_position -= 0x100000000
         
    if present_position >= 100000 or present_position <= -100000:
        print(f"Approaching limit at position {present_position}, stopping.")
        packetHandler.write4ByteTxRx(portHandler, dxl_id, 600, 0)
        break
    
    while time.time() - t_start < 3.0:
        present_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
            portHandler, dxl_id, 615
        )
        if present_velocity > 0x7FFFFFFF:
            present_velocity -= 0x100000000

        t = time.time() - start_time
        time_log.append(t)
        goal_log.append(target_velocity)
        vel_log.append(present_velocity)

        print(f"Present Velocity: {present_velocity}")
        time.sleep(0.01)

# Stop servo
packetHandler.write4ByteTxRx(portHandler, dxl_id, 600, 0)

# Disable torque and close
packetHandler.write1ByteTxRx(portHandler, dxl_id, torque_on_address, 0)
portHandler.closePort()

# Plot
plt.plot(time_log, goal_log, label="Goal Velocity")
plt.plot(time_log, vel_log, label="Present Velocity")
plt.xlabel("Time (s)")
plt.ylabel("Velocity")
plt.legend()
plt.grid()
plt.savefig("plot.png", dpi=200)
plt.show()