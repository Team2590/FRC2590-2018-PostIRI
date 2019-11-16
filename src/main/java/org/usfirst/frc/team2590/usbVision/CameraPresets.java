package org.usfirst.frc.team2590.usbVision;

public class CameraPresets {

  public static int FPS;
  public static int width;
  public static int height;
  public static int exposure;
  public static int brightness;
  public static int whiteBalance;

  /**
   * Cameras information / presets
   * @param cFPS : cameras FPS
   * @param cWidth : cameras image width
   * @param cHeight : cameras image height
   * @param cExposure : cameras exposure
   * @param cBrightness : cameras brightness
   * @param cWhiteBalance : cameras White Balance
   */
  public CameraPresets(int cFPS, int cWidth, int cHeight,
      int cExposure, int cBrightness, int cWhiteBalance) {
    FPS = cFPS;
    width = cWidth;
    height = cHeight;
    exposure = cExposure;
    brightness = cBrightness;
    whiteBalance = cWhiteBalance;
  }

}
