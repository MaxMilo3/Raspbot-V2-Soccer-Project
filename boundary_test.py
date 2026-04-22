import time

# Example: replace with your Yahboom sensor/motor library
from raspbot.Raspbot_Lib import *
from McBetter_Wheel_Sports import *

bot = Raspbot()

move_forward(10)

while dir:
    # Read 4 sensors (left → right)
    s1, s2, s3, s4 = bot.read_IR_switches()

    print(s1, s2, s3, s4)

    print(s1, s2, s3, s4)

    if s1 == 0 or s2 == 0 or s3 == 0 or s4 == 0:
        stop()

        if dir == 'forward' and s1 == 0 or s2==0:
            move_backward(10)
            time.sleep(0.3)
            stop()
            rotate_right(10)
            time.sleep(0.3)
            stop()
            move_forward(10)


        elif dir == 'forward' and s4 ==0 or s3 ==0:
            move_backward(10)
            time.sleep(0.3)
            stop()
            rotate_left(10)
            time.sleep(0.3)
            stop()
            move_forward(10)

        elif dir == 'forward' and s1==0 and s2==0 and s3==0 and s4==0:
            move_backward(10)
            time.sleep(0.3)
            stop()
            rotate_right(10)
            time.sleep(2)

    
    # If ANY sensor detects black (boundary)
    

        # Decide turn direction
        #if s1 == 0 or s2 == 0:
            #move_right(10)
            #time.sleep(0.3)
        #else:
            #move_left(10)
            #time.sleep(0.3)
    #else:
        #move_forward(10)
        #time.sleep(0.3)

    #time.sleep(0.1)