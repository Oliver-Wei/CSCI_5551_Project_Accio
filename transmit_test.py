import serial
import string

def task():
    task = 0
    file_name = "task_num.dat"
    while True:
        try:
            f = open(file_name, "r")
            s = string.split(f.readline())
            if len(s) >= 3:
                task = int(s[2])
        except:
            pass
        if (task != 0):
            break
    f.close()
    print(task)
    return task 

def serial_write():
    ser = serial.Serial('/dev/ttyUSB1',57600)
    task_num = task()
    #task_num = 44
    file_name = "ready_to_go1.dat"
    f = open(file_name,"w")
    f.write("ready")
    f.close()
    ser.write(str(task_num))
    ser.read()
    file_name = "ready_to_go2.dat"
    f = open(file_name,"w")
    f.write("ready")
    f.close()
    #line = ser.readline()
    return 

serial_write()
