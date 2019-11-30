package org.usfirst.frc.team2590.routines;

import org.usfirst.frc.team2590.auto.AutoRoutine;
import org.usfirst.frc.team2590.commands.DelayedElevator;
import org.usfirst.frc.team2590.commands.DrivePath;
import org.usfirst.frc.team2590.commands.DriveStraight;
import org.usfirst.frc.team2590.commands.Turn;
import org.usfirst.frc.team2590.robot.Robot;

import edu.wpi.first.wpilibj.Timer;

public class NullCubeRight extends AutoRoutine {

  private DrivePath dropCubeClose;
  private DelayedElevator releaseEle;
  private DriveStraight driveToTheNull;
  private DriveStraight backUp;
  private DriveStraight goIn;
  private LeftScaleFromLeft getCubeOn;
  private Turn correct;
  private DriveStraight driveToScale;
  
  @Override
  public String getKey() {
    // TODO Auto-generated method stub
    return "Null Cube Right";
  }

  @Override
  public void run() {
    correct = new Turn(-90 , false , false);
    if(getScalePos() == 'R') { 
      driveToScale = new DriveStraight(320, false, 0);
      dropCubeClose = new DrivePath("NullTerr", false, 0, false);
      backUp = new DriveStraight(-15, false, 0);
      releaseEle = new DelayedElevator(1, 80);
      Robot.getIntake().autonSucc();
      
      runCommand(releaseEle, true);
      runCommand(driveToScale, false);
      Robot.getElevator().moveSmooth(80, 40, 20);
      Timer.delay(.35);
      correct = new Turn(-90 , false , false);
      runCommand(correct, false);
      Timer.delay(.35);
      Robot.getIntake().outakeAuto();
      Timer.delay(.5);
      runCommand(backUp, false);
      Robot.getIntake().stop();
      Robot.getDriveTrain().setStop();
    } else {
      
      if(getSwitchPos() == 'R') {
        driveToScale = new DriveStraight(105, true,0);

        //dropCubeClose = new DrivePath("leftSwitchFromLeft", false, 0, true);
        Robot.getElevator().moveSmooth(30, 20, 10);
        runCommand(driveToScale, false);
        Timer.delay(.35);
        runCommand(correct, false);

        Robot.getDriveTrain().teleopDrive(-0.5, 0);
        Timer.delay(1);
        Robot.getIntake().outakeAuto();
        Robot.getDriveTrain().setStop();
      } else {
        driveToTheNull = new DriveStraight(180, false, 0);
        runCommand(driveToTheNull, false);
        Robot.getElevator().moveSmooth(30, 10, 2); 
      }
      /*
        getCubeOn = new LeftScale();
        getCubeOn.run(); */
      }
    
    }
  

}
