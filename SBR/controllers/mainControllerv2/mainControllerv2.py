from controller import Robot, Keyboard, Supervisor,Emitter,Receiver,PositionSensor
from math import atan, pi, acos, sqrt, sin, cos
import numpy as np

robot = Robot()
TIMESTEP = int(robot.getBasicTimeStep())
timestep = TIMESTEP

X,Y,Z = 0,0,0
angle = 0
maxSpeed = 15 #30
theta = [0,0,0]
otheta = [0,0,0]
para = [0,0,0]

#90,3,180 works
para[0] = 50
para[1] = 1 #1.5 #35*maxSpeed/3600
para[2] = 10#4

def setSpeed(l,r):
    motor[0].setVelocity(l)
    motor[1].setVelocity(r)


def getSpeed(ang, ang_dot):
  
    target = 0
    theta[0] = ang
    theta[1] += ang
    theta[2] = ang_dot

    if abs(theta[0])<0.05:
        theta[1] = 0
        
    speed = theta[0]*para[0]+theta[1]*para[1]+theta[2]*para[2]
    
    otheta[0] = theta[0]
    otheta[1] = theta[1]
    otheta[2] = theta[2]
    
    if abs(speed)>maxSpeed:
        speed = maxSpeed*(speed/abs(speed))
        
    return speed

state = [0, 0, 0, 0]
K = [10, -1.7, 150, 600]
#K = [6, -0.8, 200, 350]  
#K = [7.5, -0.85, 75, 200]   
#K = [10, 0, 100, 300]  # is stable to some extent. Tested for 15 minutes

def lqrSpeed(phi, dphi,theta, theta_dot):
    state[0] = phi
    state[1] = dphi
    state[2] = theta 
    state[3] = theta_dot
    
    d = [ K[i]*state[i] for i in range(len(K))]
    s = sum(d)
    
    if abs(s)>maxSpeed:
        s = maxSpeed*(s/abs(s))
    return s
 

def reset_param():
    global gAng,aAng,angle,lDisRef,rDisRef,lEnc,rEnc,oldxD,speed,oT,xD
    gAng = 0
    aAng = 0
    angle = 0
   
    for i in range(len(motorNames)):
        motor.append(robot.getDevice(motorNames[i]))
        motor[i].setPosition(float('inf'))
        motor[i].setVelocity(0)
        
        
    lDisRef = lEnc.getValue()
    rDisRef = rEnc.getValue()    
    oldxD = (lDisRef+rDisRef)*90/pi
    speed = 0
    oT = robot.getTime()
    xD = 0
    
#Setup motors
motorNames = ['leftMotor', 'rightMotor']
motor = []
   
for i in range(len(motorNames)):
    motor.append(robot.getDevice(motorNames[i]))
    motor[i].setPosition(float('inf'))
    motor[i].setVelocity(0)

#Setup Accelerometer and Gyroscope
accMeter = robot.getDevice('accel')
accMeter.enable(TIMESTEP)
gyro = robot.getDevice('gyro')
gyro.enable(TIMESTEP)

#Setup Encoders
lEnc = robot.getDevice("leftEncoder")
lEnc.enable(TIMESTEP)
rEnc = robot.getDevice("rightEncoder")
rEnc.enable(TIMESTEP)
robot.step(timestep)

#Initiate keyboard 
keyb = Keyboard()
keyb.enable(TIMESTEP)

#Initialize Emitter and Receiver
emitter = robot.getDevice("emitter")
emitter.setChannel(2)

reciever = robot.getDevice("receiver")
reciever.enable(timestep)
reciever.setChannel(1)

#Establish Proper Sync Between Emitter And Reciever


oT = robot.getTime()
gAng = 0
oldxD = 0
oldT = oT

# uses PID or else LQR
pid = False

fitness = 0

# Get reference value of encoder
lDisRef = lEnc.getValue()
rDisRef = rEnc.getValue()

stop = False
oldG = np.array([0,0,0])
xD = 0
previous_msg = ""
while robot.step(timestep) != -1:
    # Data Transmission
        # 1.Get LQR Parameters and Run
        # 2.Send Back Fitness when requested

    if reciever.getQueueLength()>0:
        message = reciever.getData().decode('utf-8')
        if message=="return_fitness":
            emitter.send(str(fitness).encode('utf-8'))
            speed = 0
            for i in range(len(motorNames)):
                motor[i].setPosition(float('inf'))
                motor[i].setVelocity(0)

            reset_param()
            stop = True
            print("Robot: Fitness for ",K," is ",fitness)
            
        else:
            
            message_ = [float(param) for param in message.split(',') ]
            K = message_
            fitness = 0
            #print(K)
            stop = False
        reciever.nextPacket()
        

    # Run Controll Algorithm  

    [x,y,z] = accMeter.getValues()
    [gx,gy,gz] = np.array(gyro.getValues())
    gAng += (gx*(robot.getTime()-oT))*180/pi
    oT = robot.getTime()
    if gy!=0:
        aAng = -atan(gz/gy)*180/pi
    else:
        aAng = 0
    if aAng>0:
        aAng = 90-aAng
    else:
        aAng = -1*(90+aAng)
    angle = 0.99*gAng + 0.01*aAng

    # Controller
    
    if pid:
        #print('%.2f'%gAng," vs ",'%.2f'%aAng, " VS ", '%.2f'%angle)
        speed = getSpeed(angle,gx)
    
    elif not(stop) :
        
        lDis = lEnc.getValue() - lDisRef
        rDis = rEnc.getValue() - rDisRef
        xD = (lDis+rDis)*90/pi
        
        if (robot.getTime()-oldT)>0:
            speed = lqrSpeed(xD,(oldxD-xD)/(robot.getTime()-oldT),angle, gx)
        else:
            speed = lqrSpeed(xD,0,angle, gx)
        oldxD = xD
        oldT = robot.getTime()
        """
        print('Theta %.3f'%angle, "\tSpeed : ",'%.3f'%speed,
        "\tIntegral : ",'%.3f'%theta[1],  "\tDerivative : ",'%.3f'%theta[2],
        "\tPhi : ",'%.3f'%xD)
        """
        
    elif stop:
        speed = 0
      
    if  abs(angle)<5:
        fitness += 0.5
        
    elif abs(angle)<50:
        fitness += 1.5
    
    
    # Penalize when the SBR falls and favour oscillations
    if abs(angle)>50:
        fitness += (100)
    #print(speed)
    setSpeed(speed,speed) 

    

     

print("Got Out")
        
    
    
    
    
    
    
