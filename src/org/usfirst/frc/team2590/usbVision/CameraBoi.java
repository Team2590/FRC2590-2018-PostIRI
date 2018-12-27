package org.usfirst.frc.team2590.usbVision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

public class CameraBoi implements CameraSettings {

  //singleton instance
  private static CameraBoi visionInstance = null; 
  public static CameraBoi getVisionInstance() {
    if(visionInstance == null) {
      visionInstance = new CameraBoi();
    }
    return visionInstance;
  }
  
  /**
   * 
   * @author Connor_Hofenbitzer
   * a holder class for x and y coordinates
   */
  private static class Target {
    //coordinates
    double cY; 
    double cX;
    
    public Target() {
      cY = 0;
      cX = 0;
    }
    
    /**
     * Sets the coordinates of the target
     * @param newX : new X coordinate
     * @param newY : new Y coordinate
     */
    public void setCoord(double newX, double newY) {
      cX = newX;
      cY = newY;
    }
    
    /**
     * Returns the targets x value
     * @return : x coordinate 
     */
    public double getX() {
      return cX;
    }
    
    /**
     * Returns the targets y value
     * @return : y coordinate
     */
    public double getY() {
      return cY;
    } 
  }
  static boolean inited = false;
  static NetworkTableInstance tableInst = NetworkTableInstance.getDefault();
  static NetworkTable contours;
  
  // find stats of targets
  private static Target focus = new Target();
  private static double[] nullArr = {140};
  
  // default angles (if network table fails, will return zeros)
  private static int targetsSeen = 0;
  static double[] xVals = new double[2];
  static double[] yVals = new double[2];
  
  // This updates the x and y values once, also sets the BoundBox coordinates
  public static void update() {
    try {
      if(!inited) {
        contours = tableInst.getTable(TABLE_NAME);
        inited = true;
      }
      
      NetworkTableEntry xVal = contours.getEntry("centerX");
      NetworkTableEntry yVal = contours.getEntry("centerY");
      xVals = xVal.getDoubleArray(nullArr); // x pos of each target
      yVals = yVal.getDoubleArray(nullArr); // y pos of each target
      
      targetsSeen = xVals.length;
      //System.out.println("x " + xVals[0] + " y " + yVals[0]);
      if(targetsSeen != 0) {
        focus.setCoord(xVals[0], yVals[0]);
      }
     
      
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  static double heightDiff = TARGET_HEIGHT - CAMERA_HEIGHT; // height difference in inches
  
  public static double[] getTargetCoords(int index) {
    double[] out = new double[2];
    if(index > targetsSeen) {
      return out;
    }
    System.out.println("adding coords " + xVals[index] + " " + yVals[index] );
    out[0] = xVals[index];
    out[1] = yVals[index];
    return out;
  }
  
  public static int TargetsFound() {
    try {
      update();
      return targetsSeen;
    } catch (Exception e) {
     return 0; 
    }
  }
  
  // horizontal angle to target
  public static double hAngleToTarget() {
    try {
      update();
      return ((focus.getX() - CX) * XDPP);
    } catch (Exception e) {
      DriverStation.reportError("Target Not Found!", false);
      return 0;
    }
  }

  // vertical angle to target
  public static double vAngleToTarget() {
    try {
      update();
      return ((focus.getY() - CY) * YDPP);
    } catch (Exception e) {
      DriverStation.reportError("Target Not Found", false);
      return 0;
    }
  }

  // returns the angle between the horizontal and the target
  public static double vAngleHorizontalToTarget() {
    try {
      update();
      return (CAMERA_ANGLE - vAngleToTarget());
    } catch (Exception e) {
      DriverStation.reportError("Target Not Found", false);
      return 0;
    }
  }

  // x distance to target from physical camera in INCHES
  public static double xDistanceToTarget() {
    update();
    return (heightDiff) / Math.tan(Math.toRadians(vAngleHorizontalToTarget()));
  }


 
}
