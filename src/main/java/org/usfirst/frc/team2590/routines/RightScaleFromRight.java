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

public class RightScaleFromRight extends AutoRoutine {

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
  private Turn turnLeft;
  
  private LeftScaleFromRight crossScaleToLeft;

  public RightScaleFromRight() {
    //elevator movement
    goUp = new GoToHeight(70 , 90 , 50);
    goUpA = new GoToHeight(40 , 90 , 50);
    goDown = new GoToHeight(5 , 90 , 50);

    //drive towards the corner of the scale
   // driveForward = new DriveStraight(120, false, 0);
  //  driveIntoScale = new DriveStraight(30, false);

    //get over to pick up the next box
    rezero = new Turn(0, false, false);
    turnToSale = new Turn(10, false, false);

    gitGud = new DriveTillBlock(12, 5);
    
    driveForward = new DriveStraight(165, true , 0);
    driveToBackScale = new DriveStraight(110, true, 90);
    driveIntoScale = new DriveStraight(18, false , -10);
    turnLeft = new Turn(-90.0, false, false);

    crossScaleToLeft = new LeftScaleFromRight();
  }

  public void init(boolean left) {
  //turn to the switch
    turnAround = new Turn(140* (left?-1:1), false, false);

    runnablePath = new DrivePath("HCRightScaleFromRight", true, 0, false);

  }
  @Override
  public String getKey() {
    // TODO Auto-generated method stub
    return "RIGHT SCALE";
  }

  @Override
  public void run() {
    init(getScalePos() == 'R');
    Robot.getDriveTrain().resetAllSensors();
    //drop off the first box
    Robot.getIntake().autonSucc();

    if(getScalePos() == 'R') {
      
      //Code For Straight Scale From Right
      Robot.getElevator().moveSmooth(70, 90, 50);
      runCommand(goUp, true);
      runCommand(runnablePath, false);
      Robot.getDriveTrain().setStop();
      Robot.getIntake().spit();
      
    } else {
      
      //Runs the Cross Scale Routine Starting From Right Position to Left Scale
      crossScaleToLeft.run();

    }
    
    Timer.delay(.125);
    

   

  }

  @Override
  public void endAuto () {
    runnablePath.cancel();
    Robot.getDriveTrain().setStop();
  }

}
