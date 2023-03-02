
import I2C_LCD_driver
from time import *

class LCDdisplay:

    def displayString(self, long_string):
        mylcd = I2C_LCD_driver.lcd()
        
        #String splitting
        stringLen = len(long_string)
        
        if stringLen > 16:
            top_string = long_string[0:16]
            bottom_string = long_string[16:]
            
        else:
            top_string = long_string[0:stringLen//2]
            bottom_string = long_string[stringLen//2:]
        
        #Display printing
        mylcd.lcd_display_string(top_string, 1)
        mylcd.lcd_display_string(bottom_string, 2)
