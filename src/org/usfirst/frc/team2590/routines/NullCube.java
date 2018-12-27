package org.usfirst.frc.team2590.routines;

import org.usfirst.frc.team2590.auto.AutoRoutine;
import org.usfirst.frc.team2590.commands.DelayedElevator;
import org.usfirst.frc.team2590.commands.DrivePath;
import org.usfirst.frc.team2590.commands.DriveStraight;
import org.usfirst.frc.team2590.commands.Turn;
import org.usfirst.frc.team2590.robot.Robot;

import edu.wpi.first.wpilibj.Timer;

public class NullCube extends AutoRoutine {

  private DrivePath dropCubeClose;
  private DelayedElevator releaseEle;
  private DriveStraight driveToTheNull;
  private DriveStraight backUp;
  private DriveStraight goIn;
  private LeftScaleFromLeft getCubeOn;
  private Turn correct;
  
  @Override
  public String getKey() {
    // TODO Auto-generated method stub
    return "Null Cube";
  }

  @Override
  public void run() {
    correct = new Turn(90 , false , false);
    
    if(getScalePos() == 'L') { 
      dropCubeClose = new DrivePath("NullTerr", true, 0, true);
      backUp = new DriveStraight(-15, false, 0);
      releaseEle = new DelayedElevator(2, 80);
      Robot.getIntake().autonSucc();
      goIn = new DriveStraight(5, false, 0);
      correct = new Turn(45 , false , false);

      runCommand(releaseEle, true);
      runCommand(dropCubeClose, false);
      runCommand(goIn, false);
      Timer.delay(.35);
      Robot.getIntake().outakeAuto();
      Timer.delay(.5);
      runCommand(backUp, false);
      Robot.getElevator().moveSmooth(0, 40, 20);
      Robot.getIntake().stop();
    } else {
      
      if(getSwitchPos() == 'L') {
        dropCubeClose = new DrivePath("leftSwitchFromLeft", false, 0, false);
        Robot.getElevator().moveSmooth(30, 20, 10);
        runCommand(dropCubeClose, false);
        Robot.getDriveTrain().teleopDrive(-0.5, 0);
        Timer.delay(.5);
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
