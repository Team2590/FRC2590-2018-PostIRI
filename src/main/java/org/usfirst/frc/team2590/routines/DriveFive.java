package org.usfirst.frc.team2590.routines;

import org.usfirst.frc.team2590.auto.AutoRoutine;
import org.usfirst.frc.team2590.commands.DriveStraight;
import org.usfirst.frc.team2590.robot.Robot;

public class DriveFive extends AutoRoutine {

  private DriveStraight drive;
  
  @Override
  public String getKey() {
    // TODO Auto-generated method stub
    return "Drive Five";
  }

  @Override
  public void run() {
    drive = new DriveStraight(100, false, 0);
    Robot.getElevator().moveSmooth(40, 20, 10);
    runCommand(drive, false);
  }

}
