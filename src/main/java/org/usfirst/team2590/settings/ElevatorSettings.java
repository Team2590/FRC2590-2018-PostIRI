package org.usfirst.team2590.settings;

public interface ElevatorSettings {

  //bang bang
  public final static double ELEVATOR_CONTROL_VOLT = 0.5;
  public final static double ELEVATOR_CONTROL_TOLERANCE = 0.5;

  public int AVERAGING_QUEUE_LENGTH = 50;

  public final static double MAX_HEIGHT = 85.0;
  public final static double MIN_HEIGHT = 0.0;

  public final static double ELEVATOR_CONSTANT = 0.3;


  //motion profile
  public final static double ELEVATOR_KP = 0.05;
  public final static double ELEVATOR_KF = 0.04;
  public final static double ELEVATOR_KA = 0.0006;


}
