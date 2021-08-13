import threading
#from queue import Queue
import time, math
import random
import logging
from gyro.mpu9250_i2c import *
from stepback import Stepback

lock = threading.Lock()
count = 0
logging.basicConfig(level=logging.DEBUG, format='(%(threadName)-9s) %(message)s',)

class Counter(object):
    
    gyro_array = []
    average_gyro_z = 0
    
    angle_array = []
    average_angle = 0
    
    def __init__(self, start=0):
        self.lock = threading.Lock()
        self.value = start
        self.right_engine = 100
        self.left_engine = 100
    
    def print_time(self):
        while True:
            #print("Do nothing")
            time.sleep(1)
        #logging.debug("Waiting for a lock")
        #self.lock.acquire()
        #try:
        #    logging.debug("Acquired a lock")
        #    self.value = self.value + 1
        #finally:
        #    logging.debug("released a lock")
        #    self.lock.release()
            
    def gyro(self, robot):
        #logging.debug("In gyro func")
        t0 = time.time()
        start_bool = False # boolean for connection
        while (time.time()-t0)<5: # wait for 5-sec to connect to IMU
            try:
                
                start_bool = True # True for forthcoming loop
                break 
            except:
                continue
        #
        #############################
        # Strings for Units/Labs
        #############################
        #
        imu_devs   = ["ACCELEROMETER","GYROSCOPE","MAGNETOMETER"]
        imu_labels = ["x-dir","y-dir","z-dir"]
        imu_units  = ["g","g","g","dps","dps","dps","uT","uT","uT"]
        #
        #############################
        # Main Loop to Test IMU
        #############################
        #
        while True:
            if start_bool==False: # make sure the IMU was started
                print("IMU not Started, Check Wiring") # check wiring if error
                break
            ##################################
            # Reading and Printing IMU values
            ##################################
            #
            try:
                ax,ay,az,wx,wy,wz = mpu6050_conv() # read and convert mpu6050 data
                mx,my,mz = AK8963_conv() # read and convert AK8963 magnetometer data
            except:
                continue 
            #
            ##################################
            # Reading and Printing IMU values
            ##################################
            #
            
            #logging.debug("Waiting for a lock")
            self.lock.acquire()
            try:
                #logging.debug(f"Acquired a lock{self.right_engine}")
                #self.value = self.value + 1
                variant = 1
                right_defiation_point = -0.5
                left_defiation_point = 0.5
                deviation = 0.665374755859375
                
                if math.atan2(mx, my) >= 0:
                    angle = math.atan2(mx, my) * (180 / math.pi)
                else:
                    angle = (math.atan2(mx, my) + 2 * math.pi) * (180 / math.pi)
                angle = int(angle)
                
                self.angle_array.append(angle)
                if len(self.angle_array) > 7:
                    self.angle_array.pop(0)
                    self.average_angle = int(sum(self.angle_array)/len(self.angle_array))
                
                if variant == 1 and robot.current_message == "forward":
                    # left deviation
                    
                    wz = wz + deviation - 2
                    
                    self.gyro_array.append(wz)
                    if len(self.gyro_array) > 4:
                        self.gyro_array.pop(0)                       
                    
                    self.average_gyro_z = sum(self.gyro_array)/len(self.gyro_array)
                        
                    if wz > left_defiation_point and self.left_engine < 100:
                        self.left_engine = self.left_engine + 1
                        robot.setDcPower(self.right_engine, self.left_engine)
                    elif wz > left_defiation_point and self.left_engine == 100 and self.right_engine > 0:
                        self.right_engine = self.right_engine - 1
                        robot.setDcPower(self.right_engine, self.left_engine)
                    #right deviation    
                    elif wz < right_defiation_point and self.right_engine < 100:
                        self.right_engine = self.right_engine + 1
                        robot.setDcPower(self.right_engine, self.left_engine)
                    elif wz < right_defiation_point and self.right_engine == 100 and self.left_engine > 0:
                        self.left_engine = self.left_engine - 1
                        robot.setDcPower(self.right_engine, self.left_engine)
                elif robot.current_message == "forward" and variant == 2:                    
                                    #self.value = self.value + 1
                    if wz > 3 and self.right_engine < 100:
                        self.right_engine = self.right_engine + 10
                        robot.setDcPower(self.right_engine, self.left_engine)
                    elif wz > 3 and self.right_engine == 100 and self.left_engine > 0:
                        self.left_engine = self.left_engine - 10
                        robot.setDcPower(self.right_engine, self.left_engine)
                    # left deviation
                    elif wz < -3 and self.left_engine < 100:
                        self.left_engine = self.left_engine + 10
                        robot.setDcPower(self.right_engine, self.left_engine)
                    elif wz < -3 and self.left_engine == 100 and self.right_engine > 0:
                        self.right_engine = self.right_engine - 10
                        robot.setDcPower(self.right_engine, self.left_engine)
                
            finally:
                #logging.debug(f"released a lock {self.right_engine} x {self.right_engine}")
                self.lock.release()


            logging.debug(50*"-")
            for imu_ii,imu_val in enumerate([ax,ay,az,wx,wy,wz,mx,my,mz]):
                if imu_ii%3==0:
                    print(20*"_"+"\n"+imu_devs[int(imu_ii/3)]) # print sensor header
                #
                ###############
                # Print Data
                ###############
                #
                logging.debug("{0}: {1:3.2f} {2}".format(imu_labels[imu_ii%3],imu_val,imu_units[imu_ii]))
  
                logging.debug(f"[{robot.current_message}]; RIGHT: {self.right_engine}; LEFT: {self.left_engine}")
                logging.debug(f"[{robot.current_message}]; ANGLE: {angle}; AVE: {self.average_angle} {self.angle_array}")
                logging.debug(f"[{robot.current_message}]; AVERAGE: {self.average_gyro_z}")
            time.sleep(0.5) # wait between prints

def oldworker(target):
    target.print_time()

def worker(target):
    target.run()

def gyro_worker(c, robot):
    #logging.debug("Gyro worker started")
    c.gyro(robot)

if __name__ == '__main__':

    counter = Counter()
    robot = Stepback()
   # for i in range(2):
    #t = threading.Thread(target=worker, args=(robot,))
    t = threading.Thread(target=gyro_worker, args=(counter, robot,))
    t.start()
    a = threading.Thread(target=worker, args=(robot,))
    a.start()
        
    #logging.debug('Waiting for the worker threads')
    main_thread = threading.currentThread()
    for t in threading.enumerate():
        if t is not main_thread:
            t.join()
        #logging.debug('Counter: %d', counter.right_engine)
        
#mytime = time.time()
#print(mytime)
#try:
#x = threading.Thread(target=print_time, args=("Thread-1", 2,))    
#y = threading.Thread(target=print_time, args=("Thread-2", 4,))
#x.start()
#y.start()
    
#except:
#
#    print("Error: unable to start thread")  
