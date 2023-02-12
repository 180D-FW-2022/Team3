class Robot:
    def __init__(self):
        self.position_x = 0
        self.position_y = 0
        self.rotation = 0

    def setRotation(self, rot):
        self.rotation = rot
    
    def rotate90CW(self):
        self.rotation += 90

    def rotate90CCW(self):
        self.rotation -= 90

    def rotate(self, rot):
        self.rotation += rot

    def move(self, distance):
        if(self.rotation == 0):
            current_robotY += int(distance)
            print(f"Current Position: x:{current_robotX}, y:{current_robotY}")
        elif(self.rotation == 90):
            current_robotX -= int(distance)
            print(f"Current Position: x:{current_robotX}, y:{current_robotY}")
        elif(self.rotation == -90):
            current_robotX += int(distance)
            print(f"Current Position: x:{current_robotX}, y:{current_robotY}")
        elif(self.rotation == 180):
            current_robotY -= int(distance)
            print(f"Current Position: x:{current_robotX}, y:{current_robotY}")
        
    def setPosition_xy(self, x, y):
        self.position_x = x
        self.position_y = y
    
    def setPosition_x(self,x):
        self.position_x = x

    def setPosition_y(self,y):
        self.position_x = y

    def getPosition_xy(self):
        return (self.position_x, self.position_y)
    

