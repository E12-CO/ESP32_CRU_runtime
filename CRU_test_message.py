# Test messages for CRU auto robot
import socket
import time

sock = socket.socket()

HOST = "192.168.0.123"
PORT = 80

sock.connect((HOST,PORT))

print("Entering the parking spot")

# Walk forward เดินหน้าจนถึงจุดจอดสีส้ม
message = "01rc210lc2100005454\n"
#message = "01rc190lc1900002525\n"
sock.send(bytes(message,'ascii'))
sock.close()

# 4
time.sleep(4)

#Turn right เลี้ยวขวาจะเข้าจอด
sock = socket.socket()
sock.connect((HOST,PORT))
message = "01rt000lc1850003232\n"
sock.send(bytes(message,'ascii'))
sock.close()

# 2.2
time.sleep(2.2)

#Jog forward เข้าขอดที่จุดสีส้ม
sock = socket.socket()
sock.connect((HOST,PORT))
message = "01rc190lc1900000505\n"
#message = "01rc210lc2100004040\n"
sock.send(bytes(message,'ascii'))
sock.close()

print("Reached the parking spot")
# 2
time.sleep(2.5)

########### รอเฉยๆจนมีคำสั่งให้เริ้ม AUTO

# Parking correction (forward at orange parking spot) หุ่นเดินหน้าจนIR ออกจากสีส้ม
sock = socket.socket()
sock.connect((HOST,PORT))
message = "01rc170lc1701000505\n"
sock.send(bytes(message,'ascii'))
sock.close()

print("Parking correction...")

time.sleep(3)

#Jog backward ถอยหลังจนล้อไปแตะที่เขียวเหลือง
sock = socket.socket()
sock.connect((HOST,PORT))
message = "01rt187lt1870001818\n"
sock.send(bytes(message,'ascii'))
sock.close()

time.sleep(2.5)

#Turn left เลี้ยวซ้ายรอเข้าหน้าไม้กัน
sock = socket.socket()
sock.connect((HOST,PORT))
message = "01rc185lt0000003332\n"
sock.send(bytes(message,'ascii'))
sock.close()

time.sleep(2.5)

################ รอระบบ Detect ไม้กั้น

print("Jog to boom bar")

#Jog forward เดินหน้าให้กล้องเห็นไม้
sock = socket.socket()
sock.connect((HOST,PORT))
message = "01rc190lc1900000505\n"
sock.send(bytes(message,'ascii'))
sock.close()

time.sleep(3)

###### เดินต่อหลังจากไม้กั้นยก

#Jog forward เดินให้พ้นจากไม้กั้น
sock = socket.socket()
sock.connect((HOST,PORT))
message = "01rc190lc1900000909\n"
sock.send(bytes(message,'ascii'))
sock.close()

print("Passed boom bar")

time.sleep(1.8)

# Turn right at boom bar เลี้ยวขวา
sock = socket.socket()
sock.connect((HOST,PORT))
message = "01rt000lc1950003232\n"
sock.send(bytes(message,'ascii'))
sock.close()

print("turned right")

time.sleep(2)

#Jog forward เดินหน้าไปอีกให้ IRเจอทางเลี้ยวซ้ายก่อนถึงไผแดง
sock = socket.socket()
sock.connect((HOST,PORT))
message = "01rc200lc2000002020\n"
sock.send(bytes(message,'ascii'))
sock.close()

time.sleep(3)

print("parking correction before red light")

# Parking correction (before red light) เดินรถจนสุดทาง
sock = socket.socket()
sock.connect((HOST,PORT))
message = "01rc160lc1601000505\n"
sock.send(bytes(message,'ascii'))
sock.close()

print("Parking corrected")

time.sleep(2)

#Jog backward ถอกกลับตั้งหลักก่อนเลี้ยว
sock = socket.socket()
sock.connect((HOST,PORT))
message = "01rt180lt1800000505\n"
sock.send(bytes(message,'ascii'))
sock.close()

time.sleep(2)

#Turn left เลี้ยวซ้ายเข้าหาไฟแดง
sock = socket.socket()
sock.connect((HOST,PORT))
message = "01rc180lt0000003232\n"
sock.send(bytes(message,'ascii'))
sock.close()

print("turned left facing red light")

time.sleep(3)