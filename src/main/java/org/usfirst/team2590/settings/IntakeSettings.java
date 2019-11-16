package org.usfirst.team2590.settings;

public interface IntakeSettings {

  //current contro
  public static double leftKF = 0.04;
  public static double leftKP = 0.0;
  public static double leftKI = 0.0;
  public static double leftKD = 0.0;

  public static double rightKF = 0.04;
  public static double rightKP = 0.0;
  public static double rightKI = 0.0;
  public static double rightKD = 0.0;

  public static double currentSuckIntake = 2; //in amps
  public static double voltageOperatingIntake = 0.7; //in percent
  public static double currentOperatingIntake = 0.0; //in amps expected

  public static double maxDistanceForEquality = 1.2;
  public static double maxDifferenceForEquality = 0.1;

  public int AVERAGING_QUEUE_LENGTH = 50;

  public double LEFT_AMP_TOL = 2;
  public double RIGHT_AMP_TOL = 2;

}
