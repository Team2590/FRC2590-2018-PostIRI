package org.usfirst.frc.team2590.robot;

public interface RobotMap {

  //drive train
	public static int leftSideDriveMID = 2;
	public static int rightSideDriveMID = 0;

	public static int leftSideDriveSID = 3;
  public static int rightSideDriveSID = 1;

	public static int leftSideEncoderA = 12;
	public static int leftSideEncoderB = 13;

	public static int rightSideEncoderA = 10;
	public static int rightSideEncoderB = 11;

	//pistons for drive
  public static int PTOSol = 3;

	//intake
  public static int leftIntakeID = 5;
  public static int rightIntakeID = 7;

  public static int leftBoxDetectorPin = 0;
  public static int rightBoxDetectorPin = 1;

	//elevator
	public static int ElevatorID = 6;

  public static int TopLimitPIN = 6;
	public static int BottomLimitPIN = 2;

	public static int ElevatorEncoderA = 14;
	public static int ElevatorEncoderB = 15;

	//climber
  public static int ClimberMID = 4;

  public static int BackPackSol = 4;
  public static int ClimberRatchetSol = 1;

  public static int climberEncoderA = 18;
  public static int climberEncoderB = 19;
}
