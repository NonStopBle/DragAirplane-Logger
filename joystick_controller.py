import pygame
class joystickController :
    def __init__ (self):
        pygame.init()
        self.joystick : any
        self.joystickList = [255]
        self.event : any
    def joyScan(self):
        joystick_count = pygame.joystick.get_count()
        for i in range(joystick_count):
                self.joystick = pygame.joystick.Joystick(i)
                self.joystick.init()
                joystick_name = self.joystick.get_name()
                target_joystick_name = "Sony Interactive Entertainment Wireless Controller"
                print(joystick_name)
                if joystick_name == target_joystick_name:
                    # print("scs")
                    print("joystick selected")
                    self.joystickList[i] = joystick_name
        
        return self.joystickList
    
    def joyMap(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def joyValueHandler(self):
        self.event = pygame.event.get()

    def joyRxValue(self) :
         if self.joystick != any :
              return self.joystick.get_axis(3)
         else :
              return None
         
    def joyRyValue(self) :
         if self.joystick != any :
              return self.joystick.get_axis(4)
         else :
              return None
         
    def joyLxValue(self) :
         if self.joystick != any :
              return self.joystick.get_axis(0)
         else :
              return None
         
    def joyLyValue(self) :
         if self.joystick != any :
              return self.joystick.get_axis(1)
         else :
              return None
         
    def joyLyPercent(self):
         return self.joyMap(self.joyLyValue() , -1 , 1 , -255 , 255)

    def joyLxPercent(self):
         return self.joyMap(self.joyLxValue() , -1 , 1 , -255 , 255)
    
    def joyRxPercent(self):
         return self.joyMap(self.joyRxValue() , -1 , 1 , -255 , 255)
    
    def joyRyPercent(self):
         return self.joyMap(self.joyRyValue() , -1 , 1 , -255 , 255)
    

# joySticker = joystickController()
# joySticker.joyScan()

# while True:
#      joySticker.joyValueHandler()
#      jy = joySticker.joyLyPercent()
#      print(jy)
