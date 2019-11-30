package org.usfirst.frc.team2590.routines;

import org.usfirst.frc.team2590.auto.AutoRoutine;
import org.usfirst.frc.team2590.commands.DrivePath;
import org.usfirst.frc.team2590.commands.GoToHeight;
import org.usfirst.frc.team2590.robot.Robot;

import edu.wpi.first.wpilibj.Timer;

public class MiddleSwitch extends AutoRoutine {

  private DrivePath driveToSwitch;
  private GoToHeight liftElevator;

  public MiddleSwitch() {
    liftElevator = new GoToHeight(30, 30 ,15);

  }

  @Override
  public String getKey() {
    // TODO Auto-generated method stub
    return "MIDDLE SWITCH";
  }

  @Override
  public void run() {
    // TODO Auto-generated method stub
    if(getSwitchPos() == 'R') {
      driveToSwitch = new DrivePath("bingMiddleRight", true, 0, true);
    } else {
      driveToSwitch = new DrivePath("bingMiddleLeft", true, 0, false);
    }
    Robot.getElevator().moveSmooth(30, 30, 15);
    runCommand(liftElevator, true);
    runCommand(driveToSwitch, false);

    Robot.getDriveTrain().teleopDrive(-0.35, 0);
    Robot.getIntake().spit();
    Timer.delay(.5);
    Robot.getDriveTrain().teleopDrive(0., 0);
  }


  @Override
  public void endAuto() {
    super.endAuto();
    driveToSwitch.cancel();
  }
}
