package org.usfirst.frc.team2590.routines;

import org.usfirst.frc.team2590.auto.AutoRoutine;
import org.usfirst.frc.team2590.commands.DrivePath;
import org.usfirst.frc.team2590.commands.GoToHeight;
import org.usfirst.frc.team2590.robot.Robot;

import edu.wpi.first.wpilibj.Timer;

public class MiddleSwitchFast extends AutoRoutine {

  private DrivePath getToSwitch;
  private GoToHeight raiseIntake;
  
  @Override
  public String getKey() {
    return "Middle Switch F";
  }

  @Override
  public void run() {
    raiseIntake = new GoToHeight(25, 60 , 30);
    
    if(getSwitchPos() == 'L') {
      getToSwitch = new DrivePath("bingMiddleLeft", true, 0, false);
    } else {
      getToSwitch = new DrivePath("bingMiddleRight", true, 0, true);
    }
    Robot.getIntake().autonSucc();
    runCommand(raiseIntake, true);
    runCommand(getToSwitch, false);
    Robot.getIntake().outakeAuto();
    Robot.getDriveTrain().teleopDrive(-0.5, 0);
    Timer.delay(.5);
   }

}
