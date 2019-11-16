package org.usfirst.frc.team2590.usbVision;

public interface CameraSettings {
  //in pixels
  public static final int IMAGE_WIDTH = 320;
  public static final int IMAGE_HEIGHT = 240;
  
  //field of view ( values from https://www.chiefdelphi.com/forums/showthread.php?t=144285)
  public static final double XFIELD = 61.0;
  public static final double YFIELD = 34.3;
  
  //center pixels 
  public static final double CX = IMAGE_WIDTH / 2.0;
  public static final double CY = IMAGE_HEIGHT / 2.0;
  
  //conversion numbers
  public static final double XDPP = XFIELD / IMAGE_WIDTH;
  public static final double YDPP = YFIELD / IMAGE_HEIGHT;
   
  //known varaibles
  public static final double CAMERA_ANGLE = 0; //in degrees
  public static final double TARGET_HEIGHT = 0; //in inches
  public static final double CAMERA_HEIGHT = 4;
  
  //the name of the table vision numbers reside in
  public static final String TABLE_NAME = "GRIP/FestusVision";
}
