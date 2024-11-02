
// Author: Mark Lanthier (SN:100000001)
//Lab 3-4
import java.awt.Point;
import java.util.ArrayList;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.Accelerometer;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.TouchSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.Robot;

public class ProjectController1 {

  private static final byte CAMERA_WIDTH = 64;
  private static final byte CAMERA_HEIGHT = 64;
  private static final double GRIPPER_MOTOR_MAX_SPEED = 0.1;

  // Various modes for the robot to be in
  static final byte WANDER = 0;
  static final byte HOME_IN = 1;
  static final byte HEAD_TO_GROUND = 2;
  static final byte HEAD_TO_RIGHT = 3;
  static final byte PIVOT = 4;
  static final byte AVOID = 5;
  static final byte PIVOT_TO_POSITION = 6;

  static final byte LEFT        = 0;
  static final byte RIGHT       = 1;
  static final byte NONE        = 2;
  static final byte FULL_SPEED  = 6;

  private static Robot robot;
  private static Motor leftMotor;
  private static Motor rightMotor;
  private static Motor gripperLift;
  private static Motor gripperLeftSide;
  private static Motor gripperRightSide;
  private static DistanceSensor leftSideSensor;
  private static DistanceSensor rightSideSensor;
  private static DistanceSensor leftAheadSensor;
  private static DistanceSensor rightAheadSensor;
  private static DistanceSensor leftAngledSensor;
  private static DistanceSensor rightAngledSensor;
  private static TouchSensor jarDetectedSensor;
  private static Compass compass;
  private static Accelerometer accelerometer;
  private static Camera camera;

  // Wait for a certain number of milliseconds
  private static void delay(int milliseconds, int timeStep) {
    int elapsedTime = 0;
    while (elapsedTime < milliseconds) {
      robot.step(timeStep);
      elapsedTime += timeStep;
    }
  }

  // Put the gripper up/down to the given position.
  // -0.0499 is "up all the way" and 0.001 is down as mush as it can
  public static void liftLowerGripper(float position) {
    gripperLift.setVelocity(GRIPPER_MOTOR_MAX_SPEED);
    gripperLift.setPosition(position);
  }

  // Put the gripper open/closed to the given position
  // 0.099 is "open all the way" and 0.01 is closed as mush as it can
  public static void openCloseGripper(float position) {
    gripperLeftSide.setVelocity(GRIPPER_MOTOR_MAX_SPEED);
    gripperRightSide.setVelocity(GRIPPER_MOTOR_MAX_SPEED);
    gripperLeftSide.setPosition(position);
    gripperRightSide.setPosition(position);
  }

  private static int getCompassReadingInDegrees(Compass compass) { 
    double bearing = 0;
    
    // FILL IN YOUR CODE HERE TO READ THE COMPASS AND CONVERT TO DEGREES
    double compassReadings[] = compass.getValues();
    double rad = Math.atan2(compassReadings[0], compassReadings[2]);
    bearing = -((rad + Math.PI) / Math.PI* 180.0);
    if(bearing > 180)bearing = 360 -bearing;
    if(bearing < -180)bearing = 360 + bearing;
    double roundedValue = Math.round(bearing/ 5) * 5;

    return (int)(roundedValue);
  }

  // This is where it all begins
  public static void main(String[] args) {
    robot = new Robot();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());

    // Set up the motors
    leftMotor = robot.getMotor("left wheel");
    rightMotor = robot.getMotor("right wheel");
    gripperLift = robot.getMotor("lift motor");
    gripperLeftSide = robot.getMotor("left finger motor");
    gripperRightSide = robot.getMotor("right finger motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0);

    // Get and enable the distance sensors
    leftSideSensor = robot.getDistanceSensor("so0");
    leftAngledSensor = robot.getDistanceSensor("so1");
    leftAheadSensor = robot.getDistanceSensor("so3");
    rightAheadSensor = robot.getDistanceSensor("so4");
    rightAngledSensor = robot.getDistanceSensor("so6");
    rightSideSensor = robot.getDistanceSensor("so7");
    leftAheadSensor.enable(timeStep);
    rightAheadSensor.enable(timeStep);
    leftAngledSensor.enable(timeStep);
    rightAngledSensor.enable(timeStep);
    leftSideSensor.enable(timeStep);
    rightSideSensor.enable(timeStep);

    // Prepare the accelerometer
    accelerometer = new Accelerometer("accelerometer");
    accelerometer.enable(timeStep);

    // Prepare the camera
    camera = new Camera("camera");
    camera.enable(timeStep);

    // Prepare the Compass sensor
    compass = robot.getCompass("compass");
    compass.enable(timeStep);

    // Prepare the jar detecting sensor
    jarDetectedSensor = new TouchSensor("touch sensor");
    jarDetectedSensor.enable(timeStep);

    // Run the robot
    byte currentMode = PIVOT;
    byte turnDirection = NONE;
    byte turnCount = 0;

    double leftSpeed, rightSpeed;
    leftSpeed = rightSpeed = 0;

    double leftCount, rightCount, centerCount;
    leftCount = rightCount = centerCount = 0;

    //openCloseGripper((float)0.099);
    

    while (robot.step(timeStep) != -1) {
      System.out.println("Current Mode: " + currentMode);
      
      //TODO Sensor values are not good

      // SENSE: Read the sensors
      boolean lsSensor = leftSideSensor.getValue() < 0.9;
      boolean laSensor = leftAngledSensor.getValue() < 0.9;
      boolean lfSensor = leftAheadSensor.getValue() < 0.9;

      boolean rsSensor = rightSideSensor.getValue() < 0.9;
      boolean raSensor = rightAngledSensor.getValue() < 0.9;
      boolean rfSensor = rightAheadSensor.getValue() < 0.9;

      boolean jarFound = jarDetectedSensor.getValue() == 1;
      System.out.println(jarFound);

      System.out.println(leftSideSensor.getValue());

      //TODO Convinced the camera does not see the cups even at height 0

      int[] image = camera.getImage();

      int r, g, b;
      r = g = b = 0;

      for (int i = 0; i <= CAMERA_WIDTH; i++) {
        r = Camera.imageGetRed(image, CAMERA_WIDTH, i, 10);
        g = Camera.imageGetGreen(image, CAMERA_WIDTH, i, 10);
        b = Camera.imageGetBlue(image, CAMERA_WIDTH, i, 10);

        

        if(i < 17){
          System.out.println("LEFT: ("+r+", "+g+", "+b+")");
          if((r < 73) && (g > 240) && (b < 99))
            leftCount++;
        }else if(i< 35){
          System.out.println("CENTER: ("+r+", "+g+", "+b+")");
          if((r < 73) && (g > 240) && (b < 99)){
            centerCount++;
          }
        }else{
          System.out.println("RIGHT: ("+r+", "+g+", "+b+")");
          if((r < 73) && (g > 240) && (b < 99)){
            rightCount++;
          }
        }

      }      

      System.out.println("LEFT: " + leftCount + " CENTER: " + centerCount + " RIGHT: " + rightCount);
      System.out.println(getCompassReadingInDegrees(compass));

      //TODO May need to add another case when robot passes under door without ball in WANDER

      // THINK: Make a decision as to what MODE to be in
      switch (currentMode) {
        case WANDER:
          // Switch to HOME_IN if ball in vision
          // In-Vision: Left/Center/Right detect something 
          if(lsSensor && laSensor || rsSensor && raSensor){
            turnDirection = NONE;
            currentMode = AVOID;
          }

          if (leftCount > 0 || centerCount > 0 || rightCount > 0) {
            turnDirection = NONE;
            currentMode = HOME_IN;
          }
          break;

        case HOME_IN:
          // Switch to HEAD_TO_GROUND if jarFound
          if (jarFound) {
            openCloseGripper((float)0.01);
            currentMode = HEAD_TO_GROUND;
          }
          break;
        
        case HEAD_TO_GROUND:
          //Switch to HEAD_TO_RIGHT if at bottom wall
          //TODO Issue since cup infront
          if(lfSensor && rfSensor){
            currentMode = PIVOT_TO_POSITION;
          }
          break;
        
        case PIVOT_TO_POSITION:
          if(getCompassReadingInDegrees(compass) == 5){
            currentMode = HEAD_TO_RIGHT;
          }
          break;
        
        case HEAD_TO_RIGHT:
          //Switch to PIVOT if left and right side sensor go off (we are at door)
          //TODO may need to backup before pivoting
          if(lsSensor && rsSensor){
            openCloseGripper((float)0.099);
            currentMode = PIVOT;
          }
          break;
        
        case PIVOT:
          //Switch to WANDER once angled the other way
          /*if(getCompassReadingInDegrees(compass) == 170 || getCompassReadingInDegrees(compass) == -175 ){
            currentMode = WANDER;
          }*/
          break;
        
        case AVOID:
          //Switch back to WANDER once sensors calmed down
          //TODO Once sensors fixed, double check this
          if(!lsSensor && !laSensor && !rsSensor && !raSensor){
            currentMode = WANDER;
          }
          break;
      }

      // REACT: Move motors according to the MODE
      leftSpeed = FULL_SPEED;
      rightSpeed = FULL_SPEED;
      switch (currentMode) {
        case WANDER: // System.out.println("Robot 1: AAAA");
        if (turnCount > 0) {
          if (turnDirection == LEFT)
            leftSpeed -= 2;
          else
            rightSpeed -= 2;
          turnCount--;
          if (--turnCount == 0) 
            turnDirection = NONE;
        }
        else {
          if ((byte)(Math.random()*5) == 0) {// turn 20% of the time
            turnDirection = (byte)(Math.random()*2);
            turnCount = (byte)(Math.random()*51+25);
          }
        }
        break;

        case HOME_IN: // System.out.println("Robot 1: BBBB");
        
        //TODO Double check w/ teamates if they have smth similar
        if(leftCount > (rightCount + 2)){
            leftSpeed-=2;
          }else if(rightCount > (leftCount + 2)){
            rightSpeed-=2;
          }
          break;
        
        case HEAD_TO_GROUND:
          
          if(getCompassReadingInDegrees(compass) < -84 && getCompassReadingInDegrees(compass) > -91){
            //Go Straight Down;
          }else{
            leftSpeed = -1 * leftSpeed / 2;
            rightSpeed = 1 * rightSpeed / 2;
          }
          break;
        
        case PIVOT: // System.out.println("Robot 1: CCCC");
        case PIVOT_TO_POSITION:
         /* leftSpeed = -1 * leftSpeed / 2;
          rightSpeed = 1 * rightSpeed / 2;
          */
          break;

        case HEAD_TO_RIGHT:
          
          if(!rsSensor){
            //Turn Right a lil
            rightSpeed--;
          }else{
            if(getCompassReadingInDegrees(compass) == 5){
              //Go Straight
            }else{
              //Pivot
              leftSpeed = -1 * leftSpeed / 2;
              rightSpeed = 1 * rightSpeed / 2;
            }
          }
          break;

          case AVOID:

            if(lsSensor && laSensor){
              rightSpeed-=4;
            }else if(rsSensor && raSensor){
              leftSpeed-=4;
            }
            break;
      }
      leftMotor.setVelocity(0);
      rightMotor.setVelocity(0);
      
      leftCount = centerCount = rightCount = 0;
    }
  }
}
