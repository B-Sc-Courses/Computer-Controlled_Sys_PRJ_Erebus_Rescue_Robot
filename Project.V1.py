#------------------In the name of Allah------------------|
#---------------Computer Controlled System---------------|
#--------------------Professor Talebi--------------------|
#------------------Erebus >Rescue Robot------------------|
#-----Mahdi Shahini 9920340 & Zahra Sedaghat 9923046-----|
#--------------------------------------------------------|
# X --> increase
# Y \/ \/ increase
#----------------------*Libraries*-----------------------|
from controller import Robot, DistanceSensor, Motor
from controller import PositionSensor, Camera
import numpy as np
import cv2                                              
import time                                                         
#--------------------------------------------------------|
#-----------------*Initialize Variables*-----------------|
MAX_SPEED = 6.28    # The max of wheel speed
PI = np.pi          # pi = 3.1415...

# e-puck Physical parameters for the kinematics model (constants)
R = 0.0205          # radius of the wheels: 20.5mm [m]
D = 0.0565          # distance between the wheels: 52mm [m]
A = 0.0500          # distance from the center of the wheels to the point of interest [m]
#--------------------------------------------------------|
#--------------*Create the Robot instance*---------------|
robot = Robot()

#--------*Get the time step of the current world*--------|
timestep = int(robot.getBasicTimeStep())     # [ms]
delta_t = robot.getBasicTimeStep()/1000.0    # [s]      
#--------------------------------------------------------|
# counter: used to maintain an active state for a number of cycles
# counter = 0
# COUNTER_MAX = 3 
#--------------------------------------------------------|



wheel_left = robot.getDevice("left wheel motor")   # Create an object to control the left wheel
wheel_right = robot.getDevice("right wheel motor") # Create an object to control the right wheel

wheel_left.setPosition(float("inf"))
wheel_right.setPosition(float("inf"))

wheel_left.setVelocity(0 * MAX_SPEED)
wheel_right.setVelocity(0 * MAX_SPEED) 

encoder = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoder.append(robot.getDevice(encoderNames[i]))
    encoder[i].enable(timestep)
    
leftEncoder = wheel_left.getPositionSensor()    #Step 1
rightEncoder = wheel_right.getPositionSensor()

leftEncoder.enable(timestep)    #Step 2
rightEncoder.enable(timestep)

phiSensor = robot.getDevice("imu")
phiSensor.enable(timestep)

colorSensor = robot.getDevice("color")
colorSensor.enable(timestep) 
def getColor():
    image = colorSensor.getImage()    # Grab color sensor camera's image view
    
    r = colorSensor.imageGetRed(image, 1, 0, 0)
    g = colorSensor.imageGetGreen(image, 1, 0, 0)
    b = colorSensor.imageGetBlue(image, 1, 0, 0)
    # print(r, g, b)
    return (r, g, b)

# distance sensor
ps = []
psNames = ['ps0', 'ps6', 'ps3', 'ps1', 'ps4', 'ps5', 'ps2']
for i in range(7):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)
psValues = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# def Distance():
#     for i in range(7):
#         psValues[i] = ps[i].getValue()
#         print(f'distance sensor output {i}: {psValues[i]:0.3f}')
# vertice detection
def wallDetected(wallValue):
    vertice = [1, 1, 1, 1, 1]
    if getColor() < (50, 50, 50):
        vertice = [0, 0, 0, 0, 0]
        print(vertice)
        return vertice
         
    if wallValue[0] < 0.055:
        vertice[0] = 0
    if wallValue[3] < 0.155:
        vertice[2] = 0
    if wallValue[6] < 0.055:
        vertice[4] = 0
    if wallValue[1] < 0.055 or wallValue[2] < 0.155:
        vertice[1] = 0
    if wallValue[5] < 0.155 or wallValue[4] < 0.055:
        vertice[3] = 0
    print(vertice)
    return vertice
#------------------------------------------------
# gps sensors
gs = robot.getDevice('gps')
gs.enable(timestep)
initpose = np.array([0.0, 0.0, 0.0])
#--------------------------------------------------------|
# X --> increase
# Y \/ \/ increase (actually is z)
#--------------------------PID for phi != 0 or 2*PI
error_last_PID, error_intg, error_dif, error_now, kPPID, kIPID, kDPID = 0, 0, 0, 0, 40, 0.05, 10

def Turn(error):
    global error_last_PID, error_intg, error_dif, error_now, kPPID, kIPID, kDPID, error_last_PID
    max = 6.28
    error_now = error
    error_intg = error + error_intg
    error_dif = error - error_last_PID
    error_last_PID = error
    rectify = (kPPID*error_now) + (kIPID*error_intg) + (kDPID*error_dif)
    
    # print(rectify)
    if error < 0.04:
        rectify = 0
        rectl = rectify
        rectr = rectify
        print(f'rectl = {rectl:0.2f}, rectr = {rectr:0.2f}')
        wheel_left.setVelocity(0)
        wheel_right.setVelocity(0)
        # END
        return 1  
    # if the error is still exist 
    # and if the velocity is bigger than Max_Speed
    if rectify > max:
        print("L>max")
        rectl = max
    rectl = rectify
    rectr = rectify
    print(f'rectl = {rectl:0.2f}, rectr = {rectr:0.2f}')
    wheel_left.setVelocity(rectl)
    wheel_right.setVelocity(-rectr) 
    # Should continue   
    return 0
#------------------------------------------------------------------------------
# ---- PI Controller for Forward movemnet
#------------------------------------------------------------------------------
error_last_PI, error_intg_PI, error_now_PI, kPPI, kIPI = 0, 0, 0, 10, 0.05
def Forward(error):
    global error_last_PI, error_intg_PI, error_now_PI, kPPI, kIPI
    error_now_PI = error
    error_intg_PI = error + error_intg_PI
    error_last_PI = error
    rectify = (kPPI*error_now_PI) + (kIPI*error_intg_PI)
    # print(rectify)
    if error < 0.005:
        rectify = 0
        rectl = rectify
        rectr = rectify
        print(f'rectl = {rectl:0.2f}, rectr = {rectr:0.2f}')
        wheel_left.setVelocity(0)
        wheel_right.setVelocity(0)
        return 1   
    max = 6.28
    if rectify > max:
        rectify = max
    rectl = rectify
    rectr = rectify
    print(f'rectl = {rectl:0.2f}, rectr = {rectr:0.2f}')
    wheel_left.setVelocity(rectl)
    wheel_right.setVelocity(rectr)   
    return 0
#------------------------------------------------------------------------------

# Angular Error

E = 0.0         #Error
def Normal_error(angle):
    if angle  < 0:
        angle = 2*PI - np.abs(angle)
    print(f'error : {angle}')
    return angle
#-------------------------------------------------------------------------------
#--------------------MAPING-----------------------------------------------------
mat=np.array([[-1,-1],[20 , -1],[-1,-1]])
CurPos=[1,0]
CurDir=1 
Continue=0

# 1 : towards Right of the map
# 2 : towards Bottom of the map
# 3 : towards Left of the map
# 4 : towars Up of the map
#SensorData=[1,0,0,1,1]

def getPrevious():
        global mat

        if mat[CurPos[0]][CurPos[1]]==21:
            return 3
            
        if mat[CurPos[0]][CurPos[1]]==22:
            return 4
            
        if mat[CurPos[0]][CurPos[1]]==23:
            return 1
        
        if mat[CurPos[0]][CurPos[1]]==24:
            return 2
        
        else:
            return 0

# Chooses where to go next
def getNextLoc(pos= CurPos):
    global mat
    global CurDir
    global CurPos

    if CurDir == 1:
        if mat[pos[0]][pos[1]+1] == 1:
                updateMesh([pos[0],pos[1]+1],0,1)
                CurDir = 1
                return 1
        
        elif mat[pos[0]+1][pos[1]]==1:
                updateMesh([pos[0]+1,pos[1]],0,1)
                #return [pos[0]+1,pos[1]] # turn right
                CurDir=4
                return 4
                
        elif mat[pos[0]-1][pos[1]]==1:
                updateMesh([pos[0]-1,pos[1]],0,1)
                #return [pos[0]-1,pos[1]] # turn left
                CurDir=2
                return 2
                
        else:
                loc = getPrevious()
                CurDir=loc
                return loc # go back
                
    
    if CurDir==2:
        if mat[pos[0]+1][pos[1]]==1:
                updateMesh([pos[0]+1,pos[1]],0,2)
                #return [pos[0]+1,pos[1]] # go straight ahead
                CurDir=2
                return 2
                
        if mat[pos[0]][pos[1]-1]==1:
                updateMesh([pos[0],pos[1]-1],0,2)
                #return [pos[0],pos[1]-1] # turn right
                CurDir=3
                return 3
                
        if mat[pos[0]][pos[1]+1]==1:
                updateMesh([pos[0],pos[1]+1],0,2)
                #return [pos[0],pos[1]+1]  # turn left
                CurDir=1
                return 1
                
        else:
                #return [pos[0]-1,pos[1]] # go back
                loc = getPrevious()
                CurDir=loc
                return loc
                
    if CurDir==3:
        if mat[pos[0]][pos[1]-1]==1:
                updateMesh([pos[0],pos[1]-1],0,3)
                #return [pos[0],pos[1]-1] # go straight ahead
                CurDir=3
                return 3
                
        if mat[pos[0]-1][pos[1]]==1:
                updateMesh([pos[0]-1,pos[1]],0,3)
                #return [pos[0]-1,pos[1]] # turn right
                CurDir=4
                return 4
                
        if mat[pos[0]+1][pos[1]]==1:
                updateMesh([pos[0]+1,pos[1]],0,3)
                #return [pos[0]+1,pos[1]]  # turn left
                CurDir=2
                return 2
                
        else:
                #return [pos[0],pos[1]+1] # go back
                loc = getPrevious()
                CurDir=loc
                return loc
    
    if CurDir==4:
        if mat[pos[0]-1][pos[1]]==1:
                updateMesh([pos[0]-1,pos[1]],0,4)
                #return [pos[0]-1,pos[1]] # go straight ahead
                CurDir=4
                return 4
                
        if mat[pos[0]][pos[1]+1]==1:
                updateMesh([pos[0],pos[1]+1],0,4)
                #return [pos[0],pos[1]+1] # turn right
                CurDir=1
                return 1
                
        if mat[pos[0]][pos[1]-1]==1:
                updateMesh([pos[0],pos[1]-1],0,4)
                #return [pos[0],pos[1]-1]  # turn left
                CurDir=3
                return 3
                
        else:
                updateMesh([pos[0]+1,pos[1]],0,4)
                #return [pos[0]+1,pos[1]] # go back
                loc = getPrevious()
                CurDir=loc
                return loc
                    
def updateMesh(pos , data , enterDir):
    global mat
    global CurPos

    if mat[pos[0]][pos[1]]==-1:
        mat[pos[0]][pos[1]]=data
        
    if mat[pos[0]][pos[1]]==1 and enterDir!=0:
        mat[pos[0]][pos[1]]=20+enterDir
        CurPos=pos
        
def updateMap(sen, pos):
    global mat
    global CurDir
    global CurPos

    xlen,ylen = mat.shape
    p=pos
    
    ###############################################
    # Case : Direction 1  - To the right of the map
    # Sensor Data 0
    if(CurDir==1):
        try:
            temp=mat[p[0]-1][p[1]]
            updateMesh([p[0]-1,p[1]] , sen[0],0)
        except:
            print("HEREEEE")
            mat=np.concatenate(([[-1]*ylen] , mat) , axis=0)
            xlen+=1
            p[0]=+1
            updateMesh([p[0]-1,p[1]] , sen[0],0)
            
        #Sensor Data 2
        try:
            temp=mat[p[0]][p[1]+1]
            updateMesh([p[0],p[1]+1],sen[2],0)
        except:
            mat=np.concatenate( (mat , np.array([[-1]*xlen]).T ), axis=1)
            ylen+=1
            # No Change in current postion
            updateMesh( [p[0] , p[1]+1] , sen[2] , 0)
        
        #Sensor Data 4
        try:
            temp=mat[p[0]+1][p[1]]
            updateMesh([p[0]+1,p[1]],sen[4],0)
        except:
            mat=np.concatenate( (mat, np.array([[-1]*ylen])), axis=0)
            xlen+=1
            # No Change in current position
            updateMesh([p[0]+1,p[1]],sen[4],0)
        
        # Sensor Data 1
        updateMesh([p[0]-1,p[1]+1] , sen[1],0)
        # Sensor Data 3
        updateMesh([p[0]+1,p[1]+1] , sen[3],0)
        
    ################################################
    # Case : Direction 2 - To the bottom of the map
    if(CurDir==2):
    # Sensor Data 0
        try:
            temp=mat[p[0]][p[1]+1]
            updateMesh([p[0],p[1]+1],sen[0],0)
        except:
            mat=np.concatenate((mat , np.array([[-1]*xlen]).T), axis=1)
            ylen+=1
            # No change in current position
            updateMesh([p[0],p[1]+1],sen[0],0)    
            
        # Sensor Data 2
        try:
            temp=mat[p[0]+1][p[1]]
            updateMesh([p[0]+1,p[1]],sen[2],0)
        except:
            mat=np.concatenate( (mat , np.array([[-1]*ylen])) , axis=0)
            xlen+=1
            # No Change in current position
            updateMesh([p[0]+1,p[1]],sen[2],0)
            
        # Sensor Data 4
        try:
            temp=mat[p[0]][p[1]-1]
            updateMesh([p[0],p[1]-1],sen[4],0)
        except:
            mat=np.concatenate((np.array([[-1]*xlen]).T , mat) , axis=1)
            ylen+=1
            p[1]=+1 
            updateMesh([p[0],p[1]-1],sen[4],0)
            
        # Sensor Data 1
        updateMesh([p[0]+1,p[1]+1],sen[1],0)
        # Sensor Data 3
        updateMesh([p[0]+1,p[1]-1],sen[3],0)
    
    ##########################################
    # Case : Direction 3
    if(CurDir==3):
    # Sensor Data 0
        try:
            temp=mat[p[0]+1][p[1]]
            updateMesh([p[0]+1,p[1]] , sen[0],0)
        except:
            mat=np.concatenate( (mat , np.array([[-1]*ylen])) , axis=0 )
            xlen+=1
            # No Change in Current position
            updateMesh([p[0]+1,p[1]] , sen[0],0)
            
        # Sensor Data 2
        try:
            temp=mat[p[0]][p[1]-1]
            updateMesh([p[0],p[1]-1] , sen[2],0)
        except:    
            mat=np.concatenate((np.array([[-1]*xlen]).T , mat) , axis=1)
            ylen+=1
            p[1]+=1
            updateMesh([p[0],p[1]-1] , sen[2],0)
            
        # Sensor Data 4
        try:
            temp=mat[p[0]-1][p[1]]
            updateMesh([p[0]-1,p[1]],sen[4],0)
        except:
            mat=np.concatenate( (np.array([[-1]*ylen]) , mat) , axis=0)
            xlen+=1
            p[0]+=1        
            updateMesh([p[0]-1,p[1]],sen[4],0)
        
        # Sensor Data 1
        updateMesh([p[0]+1, p[1]-1],sen[1],0)
        # Sensor Data 3
        updateMesh([p[0]-1, p[1]-1],sen[3],0)
    
    ######################################
    # Case : Direction 4 - To the left of the map
    # Sensor Data 0
    if(CurDir==4):
        try:
            tenp=mat[p[0]][p[1]-1]
            updateMesh([p[0] , p[1]-1],sen[0],0)
        except:
            mat=np.concatenate((np.array([[-1]*xlen]).T , mat) , axis=1)
            ylen+=1
            p[1]+=1
            updateMesh([p[0] , p[1]-1],sen[0],0)
        
        # Sensor Data 2
        try:
            tenp=mat[p[0]-1 , p[1]]
            updateMesh([p[0]-1 , p[1]],sen[2],0)
        except:
            mat=np.concatenate( (np.array([[-1]*ylen] , mat)) , axis=0)
            xlen+=1
            p[0]+=1
            updateMesh([p[0]-1 , p[1]],sen[2],0)
        
        # Sensor Data 4
        try:
            temp=mat[p[0]][p[1]+1]
            updateMesh([p[0] , p[1]+1],sen[4],0)
        except:
            mat=np.concatenate((mat, np.array([[-1]*xlen]).T) , axis=1)
            ylen+=1        
            # No Change in current position
            mat[p[0] , p[1]+1]=sen[4]
        
        # Sensor Data 1
        updateMesh([p[0]-1 , p[1]-1],sen[1],0)
        # Sensor Data 3
        updateMesh([p[0]-1 , p[1]+1],sen[3],0)
#-------------------------------------------------------------------------------
def robot_movment(Error, flag= 0): #dir_angle= PI/2, dir_old= PI
    x = 0
    if flag == 0:
        x = Forward(error=Error)
        #if dir_angle == dir_old:
            
        pass
        pass
        
    return x
#-------------------------------------------------------------------------------
def error_for(moveDir, inct, const):
     if moveDir == 1:
          return const[0] + 1 - inct[0]
     elif moveDir == 2:
          return inct[1] - const[1]
     elif moveDir == 3:
          return const[0] - inct[0]
     return const[1] - inct[1]
#-------------------------------------------------------------------------------
mapDir = {
     
}
# for the first cycle
start_state = True
movement = False

while robot.step(timestep) != -1:
    # Sampling
    phi = np.round(phiSensor.getRollPitchYaw()[2], 4)
    # Distance sensors
    for i in range(7):
        psValues[i] = ps[i].getValue()
        print(f'distance sensor output {i}: {psValues[i]:0.3f}')
    # GPS
    # initializing GPS
    GPS_NOW = np.array(gs.getValues())
    GPS_NOW = np.array([GPS_NOW[0], GPS_NOW[2]])
    if start_state :
        initpose = GPS_NOW
        start_state = False
    GPS_ABN = np.subtract(GPS_NOW, initpose)
    GPS_out = np.round(np.divide(GPS_ABN, 0.06), 2)
    print(GPS_ABN)

    if not movement:
        print("Hello")
        GPS_last = GPS_out
        CurDir_last = CurDir
        updateMap(wallDetected(psValues), CurPos)
        CurDir = getNextLoc()
        movement = True
    else:
        continueDir = CurDir - CurDir_last
        if continueDir == 0:
            y = Forward(error_for(CurDir, GPS_out, GPS_last))
        #y = robot_movment(flag= continueDir)
        if y == 1:
            print("yesss")
            y = 0
            movement = False
        pass
        
    print(CurDir)
    print(mat)
    print(CurPos)
    #------------------------------------------------------------
    pass