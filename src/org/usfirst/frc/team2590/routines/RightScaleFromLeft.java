package org.usfirst.frc.team2590.routines;

import org.usfirst.frc.team2590.auto.AutoRoutine;
import org.usfirst.frc.team2590.commands.DrivePath;
import org.usfirst.frc.team2590.commands.DriveStraight;
import org.usfirst.frc.team2590.commands.DriveTillBlock;
import org.usfirst.frc.team2590.commands.GoToHeight;
import org.usfirst.frc.team2590.commands.Turn;
import org.usfirst.frc.team2590.robot.Robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

public class RightScaleFromLeft extends AutoRoutine {

  private GoToHeight goUp;
  private GoToHeight goUpA;
  private GoToHeight goDown;

  private Turn rezero;
  private Turn turnToSale;

  private DriveStraight backUp;
  private DriveTillBlock gitGud;

  private Turn turnAround;
  private DrivePath runnablePath;
  private DriveStraight driveForward;
  private DriveStraight driveToBackScale;
  private DriveStraight driveIntoScale;
  private DriveStraight backUpBack;
  
  private Turn turnBack;
  private Turn turnRight;

  public RightScaleFromLeft() {
    //elevator movement
    goUp = new GoToHeight(70 , 90 , 50);
    goUpA = new GoToHeight(40 , 90 , 50);
    goDown = new GoToHeight(5 , 90 , 50);

    //drive towards the corner of the scale
   // driveForward = new DriveStraight(120, false, 0);
  //  driveIntoScale = new DriveStraight(30, false);

    //get over to pick up the next box
    rezero = new Turn(0, false, false);
    turnToSale = new Turn(-10, false, false);

    gitGud = new DriveTillBlock(12, 5);
    
    driveForward = new DriveStraight(161, true , 0);
    driveToBackScale = new DriveStraight(185, false, 90);
    driveIntoScale = new DriveStraight(13, false , -10);
    turnRight = new Turn(90.0, false, false);

  }

  public void init(boolean left) {
  //turn to the switch
    turnAround = new Turn(140* (left?1:-1), false, false);

    //if(left) {
      //runnablePath = new DrivePath("HCLeftScaleFromLeft", true, 0, true);
    //} else {
      //runnablePath = new DrivePath("rightScaleFromLeft", true, 0, true);
    //}

  }
  @Override
  public String getKey() {
    // TODO Auto-generated method stub
    return "LEFT CROSS SCALE";
  }

  @Override
  public void run() {
    init(getScalePos() == 'L');
    Robot.getDriveTrain().resetAllSensors();
    //drop off the first box
    Robot.getIntake().autonSucc();

  //Code For Cross Scale From Left
    Robot.getDriveTrain().resetAllSensors();
    Robot.getElevator().moveSmooth(10, 20, 10);
    runCommand(driveForward, false);
    Timer.delay(.25);
    turnRight = new Turn(90, false, false);
    runCommand(turnRight, false);
    turnRight = new Turn(90, false, false);
    runCommand(turnRight, false);
    Timer.delay(.25); 
    runCommand(driveToBackScale, false);
    
    Timer.delay(.1);
    Robot.getElevator().moveSmooth(75, 70, 60);
    turnBack = new Turn(-10, false, false);
    runCommand(turnBack, false);
    Robot.getDriveTrain().setStop();
    runCommand(driveIntoScale, false);
    Timer.delay(.15);
    Robot.getDriveTrain().setStop();
    Robot.getIntake().outakeAuto();
    
    //Timer.delay(.25);
    

   

  }

  @Override
  public void endAuto () {
    runnablePath.cancel();
    Robot.getDriveTrain().setStop();
  }

}
