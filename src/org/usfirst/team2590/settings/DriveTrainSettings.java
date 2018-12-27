package org.usfirst.team2590.settings;

public interface DriveTrainSettings {

//path following
  /*Midknight Mayhem Gains Values
  
  
  public double motionProfilekP = 0.06; //0.0925
  public double motionProfilekF = 0.013; //018 *
  public double motionProfilekA = 0.0001;//00005 *
  public double motionProfilekH = 0.0001; //0005
  */
  public double motionProfilekP = 0.06; //0.0925
  public double motionProfilekF = 0.0065; //018 *
  public double motionProfilekA = 0.0001;//00005 *
  public double motionProfilekH = 0.0001; //0005
  public double motionProfilekR = 0.0;

  //drive straight stuff
  public double maxVelStraight = 48;
  public double maxAccStraight = 20;

  public double superMaxVelStraight = 80;
  public double superMaxAccStraight =  60;
  
  public double straightProfileKP = 0.1;
  public double straightProfileKF = 0.0075;
  public double straightProfileKH = 0.0025;

  public double WheelDiameter = 4.0;
  public double WheelBaseWidth =26.0;

  public int AVERAGING_QUEUE_LENGTH = 50;
  public double velConstant = 0.015;
  public double turnKp = 0.0275; //0.035
  public double leftSideAverageTolerance = 5;
  public double rightSideAverageTolerance = 5;

  public double AccurateTurnVel = 45;
  public double AccurateTurnAcc = 20;

  public double FastTurnVel = 60;
  public double FastTurnAcc = 35;
}
