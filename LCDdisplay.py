
import I2C_LCD_driver
from time import sleep

class LCDdisplay:

    mylcd = I2C_LCD_driver.lcd()
    currentTopString = ""
    currentBottomString = ""

    def displayString(self, string1, string2=None, setCurrent=True):
        # mylcd = I2C_LCD_driver.lcd()
        #String splitting
        self.mylcd.lcd_clear()

        if string2==None:
            stringLen = len(string1)
            if stringLen > 16:
                top_string = string1[0:stringLen//2]
                bottom_string = string1[stringLen//2:]
                self.mylcd.lcd_display_string(top_string, 1)
                self.mylcd.lcd_display_string(bottom_string, 2)
                if setCurrent == True:
                    self.currentTopString = top_string
                    self.currentBottomString = bottom_string
            else:
                self.mylcd.lcd_display_string(string1, 1)
                if setCurrent == True:
                    self.currentTopString = string1
                    self.currentBottomString = ""
            sleep(1)
            return
        else:
            #Display printing
            self.mylcd.lcd_display_string(string1, 1)
            self.mylcd.lcd_display_string(string2, 2)
            if setCurrent == True:
                self.currentTopString = string1
                self.currentBottomString = string2
            sleep(1)

    def listening(self):
        self.mylcd.lcd_display_string("...", 2, 13)

    def processing(self):
        self.displayString("Processing", "voice...")

    def clear(self):
        self.mylcd.lcd_clear()

    def tryAgain(self):
        self.clear()
        # self.displayLineTwo("Please try again!")
        self.mylcd.lcd_display_string("   Please try", 1)
        self.mylcd.lcd_display_string("     again!", 2)
        sleep(1)
        # print(self.currentTopString, self.currentBottomString)
        # self.displayString(self.currentTopString, self.currentBottomString)
        # self.displayLineOne(self.currentTopString)
        # self.displayLineTwo(self.currentBottomString)
    