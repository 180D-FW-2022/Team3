class Robot:
    def __init__(self):
        self.x = 0
        self.y = 0
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

    def move(self, distance):
        if(self.rot == 0):
            self.y += int(distance)
        elif(self.rot == 90):
            self.x -= int(distance)
        elif(self.rot == -90):
            self.x += int(distance)
        elif(self.rot == 180):
            self.y -= int(distance)
        
    def setPosition_xy(self, x, y):
        self.position_x = x
        self.position_y = y
    
    def setPosition_x(self,x):
        self.position_x = x

    def setPosition_y(self,y):
        self.position_x = y

    def getPosition_xy(self):
        return (self.position_x, self.position_y)
    
    def printCurrentData(self):
        print(f"[ Robot ]: X:{self.x}, Y:{self.y}, Θ:{self.rot}")
    

