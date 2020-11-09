import threading
import time
import serial

def write_to_serial(ser, value):
    try:
        lock = threading.Lock()
        lock.acquire()
        #print("Before write")

        #print(type(value))
        #ser.flush()
        #print("value", value)
        ser.write(value.encode())
        # ser.write(1)
        time.sleep(2)
        #time.sleep(6)
        #print("After write")
        lock.release()
    except Exception as e:
        print(e)
        print("WRITE Error")
        
def read_from_arduino(ser):
    lock = threading.Lock()
    lock.acquire()
    #while (ser.in_waiting > 0):
    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            
            print("Arduino: ", line)
        #time.sleep(0.01)
        #ser.write(str(value).encode('utf-8'))
    except:
        print("READ Error")
    lock.release()



try:
    ser = serial.Serial('/dev/ttyACM0', 115200,timeout=2, write_timeout=2)
except:
    print("port not availble")
# Wait for connection to complete
time.sleep(3)

while True:
    thread_list = []
    value_to_send = "A-11.867"
    for thread in threading.enumerate():
        #print(thread.getName())
        thread_list.append(thread.getName())
        
    #print (thread_list)
    if "send" not in thread_list:
        #print("adding send thread")
        t1 = threading.Thread(target=write_to_serial, name="send", args=(ser, value_to_send,))
        t1.start()
    if "receive" not in thread_list:
        #print("adding write thread")
        
        t2 = threading.Thread(target=read_from_arduino, name="receive", args=(ser,))
        t2.start()

