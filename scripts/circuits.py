from geometry_msgs.msg import Twist
import behavior_gates as bg

def collision_avoid(behaviors):
    '''
    Fusion circuit 1
    ----------------

    This Circuit avoids collisions and assist the joystick using a homing behaviour which is invoked by the joystick
    '''
    cmd_vel = Twist()
    cmd_vel.linear.x  = behaviors[1,0]
    cmd_vel.angular.z = behaviors[1,1]
    return cmd_vel



def main_circuit_homing(behaviors):
    '''
    Fusion circuit 1
    ----------------

    This Circuit avoids collisions and assist the joystick using a homing behaviour which is invoked by the joystick
    '''
    cmd_vel = Twist()
    cmd_vel.linear.x  = bg.AND(10*behaviors[1,0],bg.INVOKE(behaviors[2,0],behaviors[0,0]))
    cmd_vel.angular.z = bg.PREVAIL(behaviors[1,1], bg.OR(behaviors[2,1],behaviors[0,1]))
    return cmd_vel

def joystick_interrupt(behaviors):
    '''
    Fusion circuit 2
    ----------------

    This Circuit lets the system navigate autonomously except if the joystick provides a different command
    '''
    cmd_vel = Twist()
    cmd_vel.linear.x  = bg.AND(behaviors[2,0], behaviors[3,0])
    cmd_vel.angular.z = bg.PREVAIL(behaviors[2,1], behaviors[3,1])
    return cmd_vel

def main_circuit(behaviors):
    '''
    Fusion circuit 3
    ----------------

    This Circuit avoids collisions and assist the joystick using a navstack behaviour which is ORed by the joystick
    '''
    cmd_vel = Twist()
    cmd_vel.linear.x  = bg.AND(10*behaviors[1,0],bg.INVOKE(behaviors[2,0],behaviors[3,0]))
    cmd_vel.angular.z = bg.PREVAIL(behaviors[1,1], bg.OR(behaviors[2,1],behaviors[3,1]))
    return cmd_vel

def navstack(behaviors):
    '''
    Fusion circuit 4
    ----------------

    This Circuit lets the navstack directly control the robot
    '''
    cmd_vel = Twist()
    cmd_vel.linear.x  = behaviors[3,0]
    cmd_vel.angular.z = behaviors[3,1]
    return cmd_vel

def nav_and_joy(behaviors):
    '''
    Fusion circuit 5
    ----------------

    This Circuit assist the joystick using a navstack behaviour which is ORed by the joystick
    '''
    cmd_vel = Twist()
    cmd_vel.linear.x  =  bg.INVOKE(behaviors[2,0],behaviors[3,0])
    cmd_vel.angular.z =  bg.OR(behaviors[2,1],behaviors[3,1])
    return cmd_vel


def sloppy_joystick(behaviors):
    '''
    Fusion circuit 6
    ----------------

    This Circuit lets the sloppy_joystick directly control the robot
    '''
    cmd_vel = Twist()
    cmd_vel.linear.x  = behaviors[2,0]
    cmd_vel.angular.z = behaviors[2,1]
    return cmd_vel
circuit_dict = {"main_circuit_homing":main_circuit_homing,"joystick_interrupt":joystick_interrupt,"main_circuit":main_circuit,"navstack":navstack,"nav_and_joy":nav_and_joy,"sloppy_joystick":sloppy_joystick,"collision_avoidance":collision_avoid}


def joystick_main_circuit(behaviors):
    '''
    Fusion circuit 7
    ----------------

    This Circuit avoids collisions and assist the joystick using a navstack behaviour which is ORed by the joystick
    '''
    cmd_vel = Twist()
    cmd_vel.linear.x  = bg.AND(10*behaviors[1,0],bg.INVOKE(behaviors[4,0],behaviors[3,0]))
    cmd_vel.angular.z = bg.PREVAIL(behaviors[1,1], bg.OR(behaviors[4,1],behaviors[3,1]))
    return cmd_vel
