from geometry_msgs.msg import Twist
import behavior_gates as bg



def circuit_1(behaviors):
    '''
    Fusion circuit 1
    ----------------

    This Circuit avoids collisions and assist the joystick using a homing behaviour which is invoked by the joystick
    '''
    cmd_vel = Twist()
    cmd_vel.linear.x  = bg.AND(10*behaviors[1,0],bg.INVOKE(behaviors[2,0],behaviors[0,0]))
    cmd_vel.angular.z = bg.PREVAIL(behaviors[1,1], bg.OR(behaviors[2,1],behaviors[0,1]))
    return cmd_vel

def circuit_2(behaviors):
    '''
    Fusion circuit 2
    ----------------

    This Circuit lets the system navigate autonomously except if the joystick provides a different command
    '''
    cmd_vel = Twist()
    cmd_vel.linear.x  = bg.PREVAIL(behaviors[2,0], behaviors[0,0])
    cmd_vel.angular.z = bg.PREVAIL(behaviors[2,1], bg.PREVAIL(behaviors[1,1],behaviors[0,1]))
    return cmd_vel

def circuit_3(behaviors):
    '''
    Fusion circuit 3
    ----------------

    This Circuit avoids collisions and assist the joystick using a navstack behaviour which is ORed by the joystick
    '''
    cmd_vel = Twist()
    cmd_vel.linear.x  = bg.AND(10*behaviors[1,0],bg.INVOKE(behaviors[2,0],behaviors[3,0]))
    cmd_vel.angular.z = bg.PREVAIL(behaviors[1,1], bg.OR(behaviors[2,1],behaviors[3,1]))
    return cmd_vel

def circuit_4(behaviors):
    '''
    Fusion circuit 4
    ----------------

    This Circuit lets the navstack directly control the robot
    '''
    cmd_vel = Twist()
    cmd_vel.linear.x  = behaviors[3,0]
    cmd_vel.angular.z = behaviors[3,1]
    return cmd_vel

def circuit_5(behaviors):
    '''
    Fusion circuit 5
    ----------------

    This Circuit assist the joystick using a navstack behaviour which is ORed by the joystick
    '''
    cmd_vel = Twist()
    cmd_vel.linear.x  =  bg.INVOKE(behaviors[2,0],behaviors[3,0])
    cmd_vel.angular.z =  bg.OR(behaviors[2,1],behaviors[3,1])
    return cmd_vel


def circuit_6(behaviors):
    '''
    Fusion circuit 4
    ----------------

    This Circuit lets the sloppy_joystick directly control the robot
    '''
    cmd_vel = Twist()
    cmd_vel.linear.x  = behaviors[2,0]
    cmd_vel.angular.z = behaviors[2,1]
    return cmd_vel
circuit_dict = {"circuit_1":circuit_1,"circuit_2":circuit_2,"circuit_3":circuit_3,"circuit_4":circuit_4,"circuit_5":circuit_5,"circuit_6":circuit_6}
