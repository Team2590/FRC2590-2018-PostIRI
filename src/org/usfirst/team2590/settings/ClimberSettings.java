package org.usfirst.team2590.settings;

public interface ClimberSettings {

  public static double MAXPREVEL = 48;
  public static double MAXPREACCEL = 50;

  public static double MAXVEL = 48;
  public static double MAXACCEL = 12;

  public static double CLIMB_HEIGHT = 17;

  public int AVERAGING_QUEUE_LENGTH = 50;

  //UP gains
  public static double UPKP = 0.1;
  public static double UPKF = 0.045;
  public static double UPKA = 0.001;

  //Low gains
  public static double LOWKP = 0.1;
  public static double LOWKF = 0.045;
  public static double LOWKA = 0.03;

  //mid gains
  public static double MIDKP = 0;
  public static double MIDKF = 0;
  public static double MIDKA = 0;

  //high gains
  public static double HIGHKP = 0;
  public static double HIGHKF = 0;
  public static double HIGHKA = 0;
}
