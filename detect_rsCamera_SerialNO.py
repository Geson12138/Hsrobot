import pyrealsense2 as rs

ctx = rs.context()
devs = ctx.query_devices() # devs是device_list类
device_num = devs.size()

print(device_num)
# 1
print(len(devs))
# 1

dev_1 = devs[0]
dev_2 = devs[1] # IndexError: list index out of range
print(type(dev_1))
print(type(dev_2))
# <class 'pyrealsense2.pyrealsense2.device'>

serial_number_1 = dev_1.get_info(rs.camera_info.serial_number)
serial_number_2 = dev_2.get_info(rs.camera_info.serial_number)
print(serial_number_1)
print(serial_number_2)
# 838212074152

