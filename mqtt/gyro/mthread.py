import threading
#from queue import Queue
import time
import random
import logging
from mpu9250_i2c import *

lock = threading.Lock()
count = 0
logging.basicConfig(level=logging.DEBUG, format='(%(threadName)-9s) %(message)s',)

class Counter(object):
    
    def __init__(self, start=0):
        self.lock = threading.Lock()
        self.value = start
        self.right_engine = 50
    
    def print_time(self):
        while True:
            logging.debug("Do nothing")
            #logging.debug("Waiting for a lock")
            #self.lock.acquire()
            #try:
            #    logging.debug("Acquired a lock")
            #    self.value = self.value + 1
            #finally:
            #    logging.debug("released a lock")
            #    self.lock.release()
            
    def gyro(self):
        logging.debug("In gyro func")
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
            
            logging.debug("Waiting for a lock")
            self.lock.acquire()
            try:
                logging.debug(f"Acquired a lock{self.right_engine}")
                #self.value = self.value + 1
                if wz > 2:
                    self.right_engine = self.right_engine + 1
                elif wz < -2:
                    self.right_engine = self.right_engine - 1
            finally:
                logging.debug(f"released a lock {self.right_engine}")
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
                logging.debug(self.right_engine)
            time.sleep(1) # wait between prints

def worker(c):
    #for i in range(20):
    #    r = random.random()
    #    logging.debug('Sleeping %0.02f', r)
    #    time.sleep(r)
    c.print_time()

def gyro_worker(c):
    logging.debug("Gyro worker started")
    c.gyro()

if __name__ == '__main__':
    counter = Counter()
   # for i in range(2):
    t = threading.Thread(target=worker, args=(counter,))
    t.start()
    a = threading.Thread(target=gyro_worker, args=(counter,))
    a.start()
        
    logging.debug('Waiting for the worker threads')
    main_thread = threading.currentThread()
    for t in threading.enumerate():
        if t is not main_thread:
            t.join()
        logging.debug('Counter: %d', counter.right_engine)
        
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