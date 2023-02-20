class Robot:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.prev_y = 0
        self.prev_x = 0
        self.leg_m_reported = 0
        self.batteryVoltage = 168000 #assumes full charge initally
        self.rot = 0
        self.in_motion = 0
        self.in_obstacle = 0


    def _calcLeastAngle(self, angle):
        to_ret = angle
        if(angle > 360):
            to_ret = angle%360
        if(angle < -360):
            to_ret = -1*(abs(angle)%360)
        if(angle == 270):
            to_ret = -90
        if(angle == -270):
            to_ret = 90
        return int(to_ret)

    def leastAngleUpdate(self):
        self.rot = self._calcLeastAngle(self.rot)

    def setRotation(self, rot):
        self.rot = rot
        self.leastAngleUpdate()
    
    def rotate90CW(self):
        self.rot += 90
        self.leastAngleUpdate()

    def rotate90CCW(self):
        self.rot -= 90
        self.leastAngleUpdate()

    def rotate(self, rot):
        self.rot += rot
        self.leastAngleUpdate()
    
    def getRotation(self):
        return self.rot
    
    def setLeg(self, dist):
        self.leg_m_reported = dist

    def getCurrentPositionInMotionXY(self):
        if(self.rot == 0):
            return (self.prev_x, self.prev_y - self.remaining_leg_m_reported)
        elif(self.rot == 90):
            return (self.prev_x + self.remaining_leg_m_reported, self.prev_y)
        elif(self.rot == -90):
            return (self.prev_x - self.remaining_leg_m_reported, self.prev_y)
        elif(self.rot == 180):
            return (self.prev_x, self.prev_y + self.remaining_leg_m_reported)

    def move(self, distance):
        self.prev_y = self.y
        self.prev_x = self.x
        if(self.rot == 0):
            self.y += int(distance)
        elif(self.rot == 90):
            self.x -= int(distance)
        elif(self.rot == -90):
            self.x += int(distance)
        elif(self.rot == 180):
            self.y -= int(distance)
        
    def setPosition_xy(self, x, y):
        self.prev_y = self.y
        self.prev_x = self.x
        self.x = x
        self.y = y

    def matchPrevWithCurrent(self):
        self.prev_y = self.y
        self.prev_x = self.x
    
    def setPosition_x(self,x):
        self.prev_y = self.y
        self.prev_x = self.x
        self.x = x

    def setPosition_y(self,y):
        self.prev_y = self.y
        self.prev_x = self.x
        self.y = y

    def getPosition_xy(self):
        return (self.x, self.y)
    
    def get_x(self):
        return self.x

    def get_y(self):
        return self.y
    
    def setBatteryVoltage(self, batVolt):
        self.batteryVoltage = batVolt

    def checkIfLowBat(self):
        if(self.batteryVoltage < 15000): #<15V low battery for 4s battery. 
            return True
        return False
    
    def printLiveData(self):
        print(f"[ Robot ]: x,y:{self.getCurrentPositionInMotionXY()}, Θ:{self.rot}")
    

