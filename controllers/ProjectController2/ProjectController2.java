// Author: Mark Lanthier (SN:100000001)
import java.awt.Point; 
import java.util.ArrayList;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.Accelerometer;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.TouchSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Lidar;

public class ProjectController2 {

  private static final byte    CAMERA_WIDTH = 64;
  private static final byte    CAMERA_HEIGHT = 64;
  private static final double  GRIPPER_MOTOR_MAX_SPEED = 0.1;
  
  // Various modes for the robot to be in
  static final byte    STRAIGHT = 0;
  static final byte    SPIN_LEFT = 1;
  static final byte    PIVOT_RIGHT = 2;
  static final byte    CURVE_LEFT = 3;
  static final byte    CURVE_RIGHT = 4;
  
  static final double  MAX_SPEED = 5;
  //private static final double  GRIPPER_MOTOR_MAX_SPEED = 0.1;
  private static final float   ADJACENCY_TOLERANCE = 0.15f; // 15cm to be considered on same wall
  private static final float   TOO_CLOSE_DISTANCE = 0.52f; // 50cm to be considered too close
  private static final float   TOO_CLOSE_SIDE_DISTANCE = 0.30f; // 25cm to be considered too close
  private static final int     WORLD_WIDTH  = 1000;
  private static final int     WORLD_HEIGHT = 1000;

  private static Robot           robot;
  private static Motor           leftMotor;
  private static Motor           rightMotor;
  private static Motor           gripperLift;
  private static Motor           gripperLeftSide;
  private static Motor           gripperRightSide;
  private static DistanceSensor  leftSideSensor; 
  private static DistanceSensor  rightSideSensor;
  private static DistanceSensor  leftAheadSensor; 
  private static DistanceSensor  rightAheadSensor;
  private static DistanceSensor  leftAngledSensor; 
  private static DistanceSensor  rightAngledSensor;
  private static TouchSensor     jarDetectedSensor;
  private static Compass         compass;
  private static Accelerometer   accelerometer;
  private static Camera          camera;
  private static float[]         rangeData;
  
  // Wait for a certain number of milliseconds
  private static void delay(int milliseconds, int timeStep) {
    int elapsedTime = 0;
    while (elapsedTime < milliseconds) {
      robot.step(timeStep);
      elapsedTime += timeStep;
    }
  }

  private static int getCompassReadingInDegrees() {
    double compassReadings[] = compass.getValues();
    double rad = Math.atan2(compassReadings[0], compassReadings[2]);
    double bearing = -((rad + Math.PI) / Math.PI * 180.0);
    if (bearing > 180)
      bearing = 360 - bearing;
    if (bearing < -180)
      bearing = 360 + bearing;
    return (int)(bearing);
  }
  
  private static void turnAway(int leftSpeed, int rightSpeed) {
    leftMotor.setVelocity(leftSpeed);
    rightMotor.setVelocity(rightSpeed);
  }

  private static void moveFrom(double x1, double y1, double x2, double y2) {
    double xDiff = x2 - x1;
    double yDiff = y2 - y1;
    double turn = Math.atan2(yDiff, xDiff) * 180 / Math.PI;
    turn = (turn - getCompassReadingInDegrees()) % 360;
    if (turn < -180)
      turn += 360;
    else if (turn > 180)
      turn -= 360;

    if (turn > 0) {
      leftMotor.setVelocity((int)(MAX_SPEED*((90-turn)/90.0)*0.85));
      rightMotor.setVelocity(MAX_SPEED);
    }
    else {//if (turn < -10) {
      leftMotor.setVelocity(MAX_SPEED);
      rightMotor.setVelocity((int)(MAX_SPEED*((90+turn)/90.0)*0.85));
    }
  }
  
  // Put the gripper up/down to the given position
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
  
  public static void resetRangeData() { 
    for (int a=0; a<360; a++)
      rangeData[a] = 0;
  }
  
  public static void applyReadings(float[] newRanges, int robotsAngle) {
    resetRangeData();
    // //// WRITE YOUR CODE HERE
     for(int a = 0; a < 180; a++){
        int angle = (robotsAngle + 90 - a);
     
        if (angle < 0){
          // //// System.out.println("angle is < 0");
          angle += 180;
         }
        else if (angle > 359){
          // //// System.out.println("angle is < 0");
          angle -= 360;
         }
       
       // System.out.println(angle);
       rangeData[Math.abs(angle)] = newRanges[a] * 100;
     }
  }

  // This is where it all begins
  public static void main(String[] args) {
      // create the Robot instance.
      // robot = new Supervisor();
    
      // Code required for being able to get the robot's location
      // Node    robotNode = robot.getSelf();
      // TranslationField = robotNode.getField("translation");
      
      // get the time step of the current world.
      int TimeStep = (int) Math.round(robot.getBasicTimeStep());
      
      // Set up the display app for the lidar
      // displayApp = new LidarDisplayApp(500, 500, robot.getDisplay("display"));
  
      // You should insert a getDevice-like function in order to get the
      // instance of a device of the robot. Something like:
      leftMotor = robot.getMotor("left wheel");
      rightMotor = robot.getMotor("right wheel");
      leftMotor.setPosition(Double.POSITIVE_INFINITY);
      rightMotor.setPosition(Double.POSITIVE_INFINITY);
      leftMotor.setVelocity(0);
      rightMotor.setVelocity(0); 
      
      // Prepare the Compass sensor
      compass = robot.getCompass("compass");
      compass.enable(TimeStep);
      
      //  Prepare the Lidar sensor
      Lidar lidar = new Lidar("Sick LMS 291");
      lidar.enable(TimeStep);
  
      //int lidarWidth = lidar.getHorizontalResolution();
      //int lidarLayers = lidar.getNumberOfLayers();
      //double maxRange = lidar.getMaxRange();
      
      /*System.out.println("Horizontal Field of View = " + lidar.getFov()*180/Math.PI); // 180
      System.out.println("Horizontal Resolution = " + lidarWidth); // 180
      System.out.println("Number of Layers = " + lidarLayers);     // 1
      System.out.println("Maximum Range = " + maxRange);           // 80*/
      
      float lidarValues[] = null;
      
      // Run the robot
      while (robot.step(TimeStep) != -1) {
        // Get the actual robot location
        double values[] = null;
        // = TranslationField.getSFVec3f();
        int x = (int)(values[0]*100) + WORLD_WIDTH/2;
        int y = -(int)(values[2]*100) + WORLD_HEIGHT/2;
        lidarValues = lidar.getRangeImage();
        applyReadings(lidarValues, getCompassReadingInDegrees());
        // applyLidarReading(lidarValues, x, y, getCompassReadingInDegrees());
  
        // WRITE YOUR CODE HERE
        
        // int indexForLeft = -1;
        // int indexForRight = -1;
        // int leftx = 0;
        // int lefty = 0;
        // int rightx = 0;
        // int righty = 0;
        // int angle = getCompassReadingInDegrees();
        // float previousDistance = lidarValues[0];
     
        // for(int i = 1; i < 180 - 1; i++){
          // if(Math.abs(lidarValues[i] - lidarValues[i+1]) > ADJACENCY_TOLERANCE){
            // leftx = (int) (previousDistance * 100 * Math.cos(Math.toRadians(angle + 90 - i-1)));
            // lefty = (int) (previousDistance * 100 * Math.sin(Math.toRadians(angle + 90 - i-1)));
            // indexForLeft = i;
            // break;
          // }
          // previousDistance = lidarValues[i];
        // }
        
        // if(indexForLeft != -1){
          // previousDistance = lidarValues[179];
          // for(int i = 179; i > 1; i--){
            // if(Math.abs(lidarValues[i] - lidarValues[i-1]) > ADJACENCY_TOLERANCE){
              // rightx = (int) (previousDistance * 100 * Math.cos(Math.toRadians(angle + 90 - i+1)));
              // righty = (int) (previousDistance * 100 * Math.sin(Math.toRadians(angle + 90 - i+1)));
              // indexForRight = i;
              // break;
            // }
            // previousDistance = lidarValues[i];
          // }
        // }
  
        
        // if(indexForRight == -1 || indexForLeft == -1){
          // displayApp.setDoorwayPoints(0,0,0,0, x, y);
        // }else if( (indexForRight != -1 && indexForLeft != -1) && 
            // (Math.abs(leftx - rightx) >= 80) && 
           // (Math.abs(leftx - rightx) <= 100)  ){
           // displayApp.setDoorwayPoints(leftx, lefty, rightx, righty, x, y);
        // }
        
        
        // int centerX = 0;
        // int centerY = 0;
        // double d = 0;
        // int px = 0;
        // int py = 0;
        // for(int i = 0; i < lidarValues.length - 1; i++){
          // d = lidarValues[i] * 100;
          // px = (int)(d * Math.cos(Math.toRadians(angle + 90 - i)));
          // py = (int)(d * Math.sin(Math.toRadians(angle + 90 - i)));
          // centerX = centerX + px;
          // centerY = centerY + py;
        // }
        
        // centerX = centerX/180; 
        // centerY = centerY/180;
        
        // centerX = centerX + x;
        // centerY = centerY + y;
      
        // displayApp.setWayPoint(centerX, centerY, x, y);
        
        
        // moveFrom(x, y, centerX, centerY);
        
        // float forward = lidarValues[90];
        // float left = lidarValues[0];
        // float right = lidarValues[179];
         
        // if(x - left < TOO_CLOSE_SIDE_DISTANCE){
          // turnAway(3, 7);
          turnAway(12, 12);
        // }else if(right - x < TOO_CLOSE_SIDE_DISTANCE){
          // turnAway(7, 3);
          turnAway(12, 12);
        // }
        // moveFrom(x, y, centerX, centerY);
        
      }
       
    }
}
