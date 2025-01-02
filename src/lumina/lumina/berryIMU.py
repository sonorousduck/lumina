#!/usr/bin/python
#
#    This program  reads the angles from the acceleromteer, gyroscope
#    and mangnetometer on a BerryIMU connected to a Raspberry Pi.
#
#    This program includes two filters (low pass and median) to improve the
#    values returned from BerryIMU by reducing noise.
#
#    The BerryIMUv1, BerryIMUv2 and BerryIMUv3 are supported
#
#    This script is python 2.7 and 3 compatible
#
#    Feel free to do whatever you like with this code.
#    Distributed as-is; no warranty is given.
#
#    http://ozzmaker.com/



import sys
import time
import math
from .IMU import IMU
import datetime
import os


RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070          # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA =  0.40              # Complementary filter constant
MAG_LPF_FACTOR = 0.4    # Low pass filter constant magnetometer
ACC_LPF_FACTOR = 0.4    # Low pass filter constant for accelerometer
ACC_MEDIANTABLESIZE = 9         # Median filter table size for accelerometer. Higher = smoother but a longer delay
MAG_MEDIANTABLESIZE = 9         # Median filter table size for magnetometer. Higher = smoother but a longer delay


################# Compass Calibration values ############
# Use calibrateBerryIMU.py to get calibration values
# Calibrating the compass isnt mandatory, however a calibrated
# compass will result in a more accurate heading value.
magXmin =  -1314
magYmin =  -1535
magZmin =  -257
magXmax =  1668
magYmax =  1319
magZmax =  3202
############### END Calibration offsets #################


#Kalman filter variables



class BerryIMU:
    def __init__(self):
        self.gyroXangle = 0.0
        self.gyroYangle = 0.0
        self.gyroZangle = 0.0
        self.CFangleX = 0.0
        self.CFangleY = 0.0
        self.CFangleXFiltered = 0.0
        self.CFangleYFiltered = 0.0
        self.kalmanX = 0.0
        self.kalmanY = 0.0
        self.oldXMagRawValue = 0
        self.oldYMagRawValue = 0
        self.oldZMagRawValue = 0
        self.oldXAccRawValue = 0
        self.oldYAccRawValue = 0
        self.oldZAccRawValue = 0

        self.a = datetime.datetime.now()



        #Setup the tables for the mdeian filter. Fill them all with '1' so we dont get devide by zero error
        self.acc_medianTable1X = [1] * ACC_MEDIANTABLESIZE
        self.acc_medianTable1Y = [1] * ACC_MEDIANTABLESIZE
        self.acc_medianTable1Z = [1] * ACC_MEDIANTABLESIZE
        self.acc_medianTable2X = [1] * ACC_MEDIANTABLESIZE
        self.acc_medianTable2Y = [1] * ACC_MEDIANTABLESIZE
        self.acc_medianTable2Z = [1] * ACC_MEDIANTABLESIZE
        self.mag_medianTable1X = [1] * MAG_MEDIANTABLESIZE
        self.mag_medianTable1Y = [1] * MAG_MEDIANTABLESIZE
        self.mag_medianTable1Z = [1] * MAG_MEDIANTABLESIZE
        self.mag_medianTable2X = [1] * MAG_MEDIANTABLESIZE
        self.mag_medianTable2Y = [1] * MAG_MEDIANTABLESIZE
        self.mag_medianTable2Z = [1] * MAG_MEDIANTABLESIZE

        self.Q_angle = 0.02
        self.Q_gyro = 0.0015
        self.R_angle = 0.005
        self.y_bias = 0.0
        self.x_bias = 0.0
        self.XP_00 = 0.0
        self.XP_01 = 0.0
        self.XP_10 = 0.0
        self.XP_11 = 0.0
        self.YP_00 = 0.0
        self.YP_01 = 0.0
        self.YP_10 = 0.0
        self.YP_11 = 0.0
        self.KFangleX = 0.0
        self.KFangleY = 0.0


        detectIMU()     #Detect if BerryIMU is connected.
        if(BerryIMUversion == 99):
            print(" No BerryIMU found... exiting ")
            sys.exit()
        initIMU()       #Initialise the accelerometer, gyroscope and compass



    def kalmanFilterY(self, accAngle, gyroRate, DT):
        y=0.0
        S=0.0


        self.KFangleY = self.KFangleY + DT * (gyroRate - self.y_bias)

        self.YP_00 = self.YP_00 + ( - DT * (self.YP_10 + self.YP_01) + self.Q_angle * DT )
        self.YP_01 = self.YP_01 + ( - DT * self.YP_11 )
        self.YP_10 = self.YP_10 + ( - DT * self.YP_11 )
        self.YP_11 = self.YP_11 + ( + self.Q_gyro * DT )

        y = accAngle - self.KFangleY
        S = self.YP_00 + self.R_angle
        K_0 = self.YP_00 / S
        K_1 = self.YP_10 / S

        self.KFangleY = self.KFangleY + ( K_0 * y )
        self.y_bias = self.y_bias + ( K_1 * y )

        self.YP_00 = self.YP_00 - ( K_0 * self.YP_00 )
        self.YP_01 = self.YP_01 - ( K_0 * self.YP_01 )
        self.YP_10 = self.YP_10 - ( K_1 * self.YP_00 )
        self.YP_11 = self.YP_11 - ( K_1 * self.YP_01 )

        return self.KFangleY

    def kalmanFilterX(self, accAngle, gyroRate, DT):
        x=0.0
        S=0.0



        self.KFangleX = self.KFangleX + DT * (gyroRate - self.x_bias)

        self.XP_00 = self.XP_00 + ( - DT * (self.XP_10 + self.XP_01) + self.Q_angle * DT )
        self.XP_01 = self.XP_01 + ( - DT * self.XP_11 )
        self.XP_10 = self.XP_10 + ( - DT * self.XP_11 )
        self.XP_11 = self.XP_11 + ( + self.Q_gyro * DT )

        x = accAngle - self.KFangleX
        S = self.XP_00 + self.R_angle
        K_0 = self.XP_00 / S
        K_1 = self.XP_10 / S

        self.KFangleX = self.KFangleX + ( K_0 * x )
        self.x_bias = self.x_bias + ( K_1 * x )

        self.XP_00 = self.XP_00 - ( K_0 * self.XP_00 )
        self.XP_01 = self.XP_01 - ( K_0 * self.XP_01 )
        self.XP_10 = self.XP_10 - ( K_1 * self.XP_00 )
        self.XP_11 = self.XP_11 - ( K_1 * self.XP_01 )

        return self.KFangleX


    def read_imu(self):

        #Read the accelerometer,gyroscope and magnetometer values
        self.ACCx = IMU.readACCx()
        self.ACCy = IMU.readACCy()
        self.ACCz = IMU.readACCz()
        self.GYRx = IMU.readGYRx()
        self.GYRy = IMU.readGYRy()
        self.GYRz = IMU.readGYRz()
        self.MAGx = IMU.readMAGx()
        self.MAGy = IMU.readMAGy()
        self.MAGz = IMU.readMAGz()


        #Apply compass calibration
        self.MAGx -= (magXmin + magXmax) /2
        self.MAGy -= (magYmin + magYmax) /2
        self.MAGz -= (magZmin + magZmax) /2


        ##Calculate loop Period(LP). How long between Gyro Reads
        b = datetime.datetime.now() - self.a
        self.a = datetime.datetime.now()
        LP = b.microseconds/(1000000*1.0)
        outputString = "Loop Time %5.2f " % ( LP )



        ###############################################
        #### Apply low pass filter ####
        ###############################################
        self.MAGx =  self.MAGx  * MAG_LPF_FACTOR + self.oldXMagRawValue*(1 - MAG_LPF_FACTOR);
        self.MAGy =  self.MAGy  * MAG_LPF_FACTOR + self.oldYMagRawValue*(1 - MAG_LPF_FACTOR);
        self.MAGz =  self.MAGz  * MAG_LPF_FACTOR + self.oldZMagRawValue*(1 - MAG_LPF_FACTOR);
        self.ACCx =  self.ACCx  * ACC_LPF_FACTOR + self.oldXAccRawValue*(1 - ACC_LPF_FACTOR);
        self.ACCy =  self.ACCy  * ACC_LPF_FACTOR + self.oldYAccRawValue*(1 - ACC_LPF_FACTOR);
        self.ACCz =  self.ACCz  * ACC_LPF_FACTOR + self.oldZAccRawValue*(1 - ACC_LPF_FACTOR);

        self.oldXMagRawValue = self.MAGx
        self.oldYMagRawValue = self.MAGy
        self.oldZMagRawValue = self.MAGz
        self.oldXAccRawValue = self.ACCx
        self.oldYAccRawValue = self.ACCy
        self.oldZAccRawValue = self.ACCz

        #########################################
        #### Median filter for accelerometer ####
        #########################################
        # cycle the table
        for x in range (ACC_MEDIANTABLESIZE-1,0,-1 ):
            self.acc_medianTable1X[x] = self.acc_medianTable1X[x-1]
            self.acc_medianTable1Y[x] = self.acc_medianTable1Y[x-1]
            self.acc_medianTable1Z[x] = self.acc_medianTable1Z[x-1]

        # Insert the lates values
        self.acc_medianTable1X[0] = self.ACCx
        self.acc_medianTable1Y[0] = self.ACCy
        self.acc_medianTable1Z[0] = self.ACCz

        # Copy the tables
        acc_medianTable2X = self.acc_medianTable1X[:]
        acc_medianTable2Y = self.acc_medianTable1Y[:]
        acc_medianTable2Z = self.acc_medianTable1Z[:]

        # Sort table 2
        acc_medianTable2X.sort()
        acc_medianTable2Y.sort()
        acc_medianTable2Z.sort()

        # The middle value is the value we are interested in
        self.ACCx = acc_medianTable2X[int(ACC_MEDIANTABLESIZE/2)];
        self.ACCy = acc_medianTable2Y[int(ACC_MEDIANTABLESIZE/2)];
        self.ACCz = acc_medianTable2Z[int(ACC_MEDIANTABLESIZE/2)];



        #########################################
        #### Median filter for magnetometer ####
        #########################################
        # cycle the table
        for x in range (MAG_MEDIANTABLESIZE-1,0,-1 ):
            self.mag_medianTable1X[x] = self.mag_medianTable1X[x-1]
            self.mag_medianTable1Y[x] = self.mag_medianTable1Y[x-1]
            self.mag_medianTable1Z[x] = self.mag_medianTable1Z[x-1]

        # Insert the latest values
        self.mag_medianTable1X[0] = self.MAGx
        self.mag_medianTable1Y[0] = self.MAGy
        self.mag_medianTable1Z[0] = self.MAGz

        # Copy the tables
        mag_medianTable2X = self.mag_medianTable1X[:]
        mag_medianTable2Y = self.mag_medianTable1Y[:]
        mag_medianTable2Z = self.mag_medianTable1Z[:]

        # Sort table 2
        mag_medianTable2X.sort()
        mag_medianTable2Y.sort()
        mag_medianTable2Z.sort()

        # The middle value is the value we are interested in
        self.MAGx = mag_medianTable2X[int(MAG_MEDIANTABLESIZE/2)];
        self.MAGy = mag_medianTable2Y[int(MAG_MEDIANTABLESIZE/2)];
        self.MAGz = mag_medianTable2Z[int(MAG_MEDIANTABLESIZE/2)];



        #Convert Gyro raw to degrees per second
        rate_gyr_x =  self.GYRx * G_GAIN
        rate_gyr_y =  self.GYRy * G_GAIN
        rate_gyr_z =  self.GYRz * G_GAIN


        #Calculate the angles from the gyro.
        self.gyroXangle+=rate_gyr_x*LP
        self.gyroYangle+=rate_gyr_y*LP
        self.gyroZangle+=rate_gyr_z*LP

        #Convert Accelerometer values to degrees
        AccXangle =  (math.atan2(self.ACCy,self.ACCz)*RAD_TO_DEG)
        AccYangle =  (math.atan2(self.ACCz,self.ACCx)+M_PI)*RAD_TO_DEG


        #Change the rotation value of the accelerometer to -/+ 180 and
        #move the Y axis '0' point to up.  This makes it easier to read.
        if AccYangle > 90:
            AccYangle -= 270.0
        else:
            AccYangle += 90.0



        #Complementary filter used to combine the accelerometer and gyro values.
        self.CFangleX=AA*(self.CFangleX+rate_gyr_x*LP) +(1 - AA) * AccXangle
        self.CFangleY=AA*(self.CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle

        #Kalman filter used to combine the accelerometer and gyro values.
        self.kalmanY = self.kalmanFilterY(AccYangle, rate_gyr_y,LP)
        self.kalmanX = self.kalmanFilterX(AccXangle, rate_gyr_x,LP)

        #Calculate heading
        heading = 180 * math.atan2(self.MAGy,self.MAGx)/M_PI

        #Only have our heading between 0 and 360
        if heading < 0:
            heading += 360

        ####################################################################
        ###################Tilt compensated heading#########################
        ####################################################################
        #Normalize accelerometer raw values.
        self.accXnorm = self.ACCx/math.sqrt(self.ACCx * self.ACCx + self.ACCy * self.ACCy + self.ACCz * self.ACCz)
        self.accYnorm = self.ACCy/math.sqrt(self.ACCx * self.ACCx + self.ACCy * self.ACCy + self.ACCz * self.ACCz)


        #Calculate self.pitch and self.roll
        self.pitch = math.asin(self.accXnorm)
        self.roll = -math.asin(self.accYnorm/math.cos(self.pitch))


        #Calculate the new tilt compensated values
        #The compass and accelerometer are orientated differently on the the BerryIMUv1, v2 and v3.
        #This needs to be taken into consideration when performing the calculations

        #X compensation
        if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
            self.magXcomp = self.MAGx*math.cos(self.pitch)+self.MAGz*math.sin(self.pitch)
        else:                                                                #LSM9DS1
            self.magXcomp = self.MAGx*math.cos(self.pitch)-self.MAGz*math.sin(self.pitch)

        #Y compensation
        if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
            self.magYcomp = self.MAGx*math.sin(self.roll)*math.sin(self.pitch)+self.MAGy*math.cos(self.roll)-self.MAGz*math.sin(self.roll)*math.cos(self.pitch)
        else:                                                                #LSM9DS1
            self.magYcomp = self.MAGx*math.sin(self.roll)*math.sin(self.pitch)+self.MAGy*math.cos(self.roll)+self.MAGz*math.sin(self.roll)*math.cos(self.pitch)
        




        #Calculate tilt compensated heading
        tiltCompensatedHeading = 180 * math.atan2(self.magYcomp,self.magXcomp)/M_PI

        if tiltCompensatedHeading < 0:
            tiltCompensatedHeading += 360


        ##################### END Tilt Compensation ########################

        # return (self.magXcomp, self.magYcomp, AccXangle, AccYangle, self.gyroXangle, self.gyroYangle, self.gyroZangle, self.CFangleX, self.CFangleY, heading, tiltCompensatedHeading, self.kalmanX, self.kalmanY)


        # if 1:                       #Change to '0' to stop showing the angles from the accelerometer
        #     outputString += "#  self.ACCx Angle %5.2f self.ACCy Angle %5.2f  #  " % (AccXangle, AccYangle)

        # if 1:                       #Change to '0' to stop  showing the angles from the gyro
        #     outputString +="\t# GRYX Angle %5.2f  self.GYRy Angle %5.2f  self.GYRz Angle %5.2f # " % (self.gyroXangle,self.gyroYangle,self.gyroZangle)

        # if 1:                       #Change to '0' to stop  showing the angles from the complementary filter
        #     outputString +="\t#  self.CFangleX Angle %5.2f   self.CFangleY Angle %5.2f  #" % (self.CFangleX,self.CFangleY)

        # if 1:                       #Change to '0' to stop  showing the heading
        #     outputString +="\t# HEADING %5.2f  tiltCompensatedHeading %5.2f #" % (heading,tiltCompensatedHeading)

        # if 1:                       #Change to '0' to stop  showing the angles from the Kalman filter
        #     outputString +="# self.kalmanX %5.2f   self.kalmanY %5.2f #" % (self.kalmanX,self.kalmanY)

        # print(outputString)

        #slow program down a bit, makes the output more readable
        # time.sleep(0.03)

if __name__ == "__main__":
    imu = BerryIMU()
    
    while True:
        imu.read_imu()


        # print(f"Magnetometer: {imu.magXcomp}, {imu.magYcomp}")
        print(f"Accelometer: {imu.accXnorm}, {imu.accYnorm}")
        
        # outputString = ""
        
        # if 1:                       #Change to '0' to stop showing the angles from the accelerometer
        #     outputString += "#  self.ACCx Angle %5.2f self.ACCy Angle %5.2f  #  " % (AccXangle, AccYangle)

        # if 1:                       #Change to '0' to stop  showing the angles from the gyro
        #     outputString +="\t# GRYX Angle %5.2f  self.GYRy Angle %5.2f  self.GYRz Angle %5.2f # " % (gyroXangle,gyroYangle,gyroZangle)

        # if 1:                       #Change to '0' to stop  showing the angles from the complementary filter
        #     outputString +="\t#  self.CFangleX Angle %5.2f   self.CFangleY Angle %5.2f  #" % (CFangleX,CFangleY)

        # if 1:                       #Change to '0' to stop  showing the heading
        #     outputString +="\t# HEADING %5.2f  tiltCompensatedHeading %5.2f #" % (heading,tiltCompensatedHeading)

        # if 1:                       #Change to '0' to stop  showing the angles from the Kalman filter
        #     outputString +="# self.kalmanX %5.2f   self.kalmanY %5.2f #" % (kalmanX,kalmanY)
        

        # print(outputString)

        
        time.sleep(0.1)