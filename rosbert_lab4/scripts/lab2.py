#!/usr/bin/env python 

#Author Joseph St. Germain 
#Co-Authur Arthur lockmans drive smooth function
# Modified by Christopher Bove for Lab 2

import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


wheel_rad = 3.5 / 100.0 #divide by 100 for meters
wheel_base = 23.0 / 100.0 #converted to meters

def publishTwist(lin_Vel, ang_Vel):
    """Send a movement (twist) message."""
    global pub
    msg = Twist()
    msg.linear.x = lin_Vel
    msg.angular.z = ang_Vel
    pub.publish(msg)


def navToPose(goal):
    """Drive to a goal subscribed to from /move_base_simple/goal"""
    haltNow = False # someone intends us to move, clear last stop command
    status = Bool()
    status = False
    status_pub.publish(status)
    #compute angle required to make straight-line move to desired pose
    global xPosition
    global yPosition
    global theta
    global pose
    #capture desired x and y positions
    desiredY = goal.pose.position.y
    desiredX = goal.pose.position.x
    #capture desired angle
    quat = goal.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    desiredT = yaw * (180.0/math.pi)
    #print our positions for debugging
    print "Actual X: %f Y: %f" % (xPosition, yPosition)
    print "Desired X: %f Y: %f" % (desiredX, desiredY)
    #calculate the change in desired-actual
    deltaX = desiredX-xPosition
    deltaY = desiredY-yPosition
    print "change in x: %f Y: %f" % (deltaX, deltaY)
    #compute distance to target using distance formula
    distance = math.sqrt(pow(deltaX,2) + pow(deltaY,2))
    print "need to move %f meters" % distance
    #compute initial turn amount: angle of new path - theta 
    print "theta: %f initialturn: %f" % (theta, math.degrees( math.atan2(deltaY, deltaX)))
    initialTurn = math.degrees( math.atan2(deltaY, deltaX)) - theta
    print "need to rotate %f initially and %f finally" % (initialTurn, desiredT)


    print "spin!" #turn to calculated angle
    rotateLocal(initialTurn)
    print "move!" #move in straight line specified distance to new pose
    driveSmooth(0.25, distance)
    #rospy.sleep(2)
    print "spin!" #spin to final angle 
    rotate(desiredT)
    print "done"
    status = True
    if not haltNow:
        status_pub.publish(status)



#This function accepts two wheel velocities (m/s) and a time interval.
def spinWheels(u1, u2, time):
    """This function accepts two wheel velocities and a time interval."""
    global pub

    #compute wheel speeds
    u = (u1+u2)/2.0 #Determines the linear velocity of base based on the wheel velocity
    w = (u2 - u1)/(wheel_base) #Determines the angular velocity of base
    start = rospy.Time().now().secs #record start time
    #create movement and stop messages
    move_msg = Twist() #creates a move_msg object inheriting type from the Twist() class
    move_msg.linear.x = u #sets linear velocity
    move_msg.angular.z = w #sets amgular velocity (Populates messages with data.)

    stop_msg = Twist()
    stop_msg.linear.x = 0
    stop_msg.angular.z = 0
    #publish move message for desired time
    while(rospy.Time().now().secs - start < time and not rospy.is_shutdown()): # waits for said time and checks for ctrl C
        pub.publish(move_msg) #publishes the move_msg
    pub.publish(stop_msg)

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    """This function accepts a speed and a distance for the robot to move in a straight line"""
    global pose

    initialX = pose.position.x
    initialY = pose.position.y
    atTarget = False
    #Loop until the distance between the attached frame and the origin is equal to the
    #distance specified 
    while (not atTarget and not rospy.is_shutdown() and not haltNow):
        currentX = pose.position.x
        currentY = pose.position.y
        currentDistance = math.sqrt(pow(currentX-initialX,2) + pow(currentY-initialY,2)) #Distance formula
        #reached current distance, stop robot and loop        
        if (currentDistance >= distance) or haltNow:
            atTarget = True
            publishTwist(0, 0)
        #not at desired position, keep moving forward
        else:
            publishTwist(speed, 0)
            rospy.sleep(0.15)


def driveSmooth(speed, distance):
    """This function accepts a speed and a distance for the robot to move in a smoothed straight line."""
    driveStraight(speed, distance)
    #TODO - implement this


def rotate(angle):
    """This rotates the robot to the angle specified in world coordinates."""
    global odom_list
    global pose
    if (angle > 180 or angle<-180):
        print "angle is too large or small"
        #TODO do something about this
    vel = Twist();   

    # calculate error
    error = angle-math.degrees(pose.orientation.z)
    #determine which way to turn based on the error
    angularZ = 0.75 # velocity for turnning
    # turn CCW if the error is positive, CW if negative
    if(error < 0):
        angularZ *= -1.0 # flip velocity

    vel.linear.x = 0 #set linear velocity to 0 (just want to rotate)
    
    #for checking if our error starts getting worse - maybe we need to spin the other way
    errorIncreases = 0
    lastError = error

    while ((abs(error) > 2) and not rospy.is_shutdown() and not haltNow):
        #start the robot's motion and determine if we are at the right angle.    
        print "theta: %d  Error: %d" % (math.degrees(pose.orientation.z), error)

        #check if our error consistently gets worse and we should flip the velocity
        #if error has grown since last time
        if(abs(error) > abs(lastError)):
            errorIncreases += 1 #mark the increase
        else:
            errorIncreases = 0
        #if we have seen increases in error, flip velocity
        if(errorIncreases > 2):
            angularZ *= -1.0
            errorIncreases = 0
        lastError = error

        vel.angular.z = angularZ #sets angular velocity 
        pub.publish(vel)
        error = angle-math.degrees(pose.orientation.z) #recalc error
        rospy.sleep(0.05)
    #once finished, stop
    vel.angular.z = 0.0
    if not haltNow:
        pub.publish(vel)

#This function is duplicated, not good practice, but rotates the robot by its local frame
def rotateLocal(angle):
    """This rotates the robot to the angle specified in local coordinates (left 90 degrees)."""
    global odom_list
    global pose
    if (angle > 180 or angle<-180):
        print "angle is too large or small... adjusting"
    #subtract off a rotation if too large or small
    #TODO deal with larger than 360 angles
    if (angle > 180):
        angle -= 360
        print "too positive: %f" % angle
    if (angle < -180):
        angle += 360
        print "too negative: %f" % angle
    vel = Twist();   

    # set rotation direction
    startingAngle = math.degrees(pose.orientation.z)
    endingAngle = startingAngle + angle
    # check if we'll travel through the +-180 singularity, if so, flip angle to other side of singularity
    passingSingularity = False
    if(endingAngle > 180):
        endingAngle -=360
        passingSingularity = True
    if(endingAngle < -180):
        endingAngle += 360
        passingSingularity = True
    #calc error
    error = angle-(startingAngle - math.degrees(pose.orientation.z))
    #determine which way to turn based on the angle
    angularZ = -0.75
    # turn positive in situations where the angles and errors are in certain ranges
    #to optimize which direction we turn
    hackyError = endingAngle - startingAngle
    if( (hackyError<0 and hackyError<-180) or (hackyError>0 and hackyError<180)):
        angularZ *= -1.0 #multiply to make velocity positive

    vel.linear.x = 0 #set linear velocity to 0
    error = endingAngle - math.degrees(pose.orientation.z)
    
    #for checking if our error starts getting worse - maybe we need to spin the other way
    errorIncreases = 0
    lastError = error

    while ((abs(error) > 2) and not rospy.is_shutdown() and not haltNow):
        #start the robot's motion and determine if we are at the right angle.    
        print "theta: %d  Error: %d EndingAngle: %d" % (math.degrees(pose.orientation.z), error, endingAngle)
        #check if our error got worse, changed direction and we should flip the velocity
        if(abs(error) > abs(lastError) and not passingSingularity):
            errorIncreases += 1
        else:
            errorIncreases = 0
        if(errorIncreases > 2):
            angularZ *= -1.0
            errorIncreases = 0
        lastError = error
        
        vel.angular.z = angularZ #sets angular velocity
        pub.publish(vel)
        error = endingAngle - math.degrees(pose.orientation.z)
        rospy.sleep(0.05)
    vel.angular.z = 0.0
    if not haltNow:
        pub.publish(vel)

def executeTrajectory():
    """This function sequentially calls methods to perform a trajectory."""
    print "Driving 60cm"
    driveStraight(0.3, 0.6) # drive forward 60cm
    print "Turning right 90"
    rotateLocal(-90) # turn right 90
    print "Driving 45cm"
    driveStraight(0.3, 0.45) # drive forward 45cm
    print "Turning left 135"
    rotateLocal(135) # turn left 135
    print "Finished trajectory"


def driveArc(radius, speed, angle):
    """This function works the same as rotate how ever it does not publish linear velocities."""
    #assuming radius is turning radius, speed is drive speed, angle is desired final angle
    #calculate wheel speeds and time to move from current pose to final pose
    #spinWheels with time and speeds to move to correct pose
    w = speed / radius
    v1 = w * (radius + .5*.352)
    v2 = w * (radius - .5*.352)

    ############################# The rest of this function will be at least as long as rotate
    pass  # Delete this 'pass' once implemented


def readBumper(msg):
    """Bumper event callback"""
    # if center button is pressed, execute the trajectory
    if ((msg.state == 1) and (msg.bumper == 1)):
        print "Bumper pressed!"
        
        #executeTrajectory()

def stopCallback(msg):
    #if 
    if msg:
        haltNow = True
        publishTwist(0, 0) #stop!
        haltNow = True

def wiggleCallback(wiggleTime):
    if wiggleTime:
        print "Wiggle Time!!!"
        # rotate robot back and forth several times
        angle = 45
        startingAngle = math.degrees(pose.orientation.z)
        for i in range(0, 7):
            rotateLocal(angle)
            rospy.sleep(1)
            rotateLocal(-angle)
            rospy.sleep(1)
        rotate(startingAngle)
        rospy.sleep(0.3)
        status_pub.publish(True)


#keeps track of current location and orientation
def tCallback(event):
    global pose
    global xPosition
    global yPosition
    global theta

    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))
    pose.position.x=position[0]
    pose.position.y=position[1]
    # the previous 2 lines and next 2 lines are repedative. Joes bad
    xPosition=position[0]
    yPosition=position[1]

    odomW = orientation
    q = [odomW[0], odomW[1], odomW[2], odomW[3]]
    roll, pitch, yaw = euler_from_quaternion(q)
    #convert yaw to degrees
    pose.orientation.z = yaw
    theta = math.degrees(yaw)

# This is the program's main function
if __name__ == '__main__':
    rospy.init_node('cbove_lab2')
    global pub
    global pose
    global odom_list
    global haltNow
    haltNow = False
    #global odom_tf
    pose = Pose()
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    goal_sub = rospy.Subscriber('/clicked_pose', PoseStamped, navToPose, queue_size=1) #callback for setting pose goal
    status_pub = rospy.Publisher('/moves_done', Bool, None, queue_size=1) #publishes when robot is done moving
    stop_sub = rospy.Subscriber('stop_move', Bool, stopCallback, queue_size=1)
    wiggle_sub = rospy.Subscriber('wiggle_move', Bool, wiggleCallback, queue_size=1)

    rospy.Timer(rospy.Duration(.01), tCallback) # timer callback for robot location
    
    odom_list = tf.TransformListener() #listner for robot location

    rospy.sleep(2)

    print "Starting Lab 2"

    spinWheels(0,0,0.5) #make sure robot is in a stopped state
    print "stopped the robot"

    while not rospy.is_shutdown():
        #spinWheels(0.5,0.5,1.0) #for unit testing functions
        #driveStraight(-0.3,0.5) 
        #rotateLocal(180.0)       
        rospy.spin()
    
    print "Lab 2 complete!"
