package org.usfirst.frc.team2590.routines;

import org.usfirst.frc.team2590.auto.AutoRoutine;
import org.usfirst.frc.team2590.commands.DriveStraight;
import org.usfirst.frc.team2590.commands.GoToHeight;
import org.usfirst.frc.team2590.commands.Turn;
import org.usfirst.frc.team2590.robot.Robot;

public class HCStraightPath extends AutoRoutine {

  private LeftScaleFromLeft driveToScale;
  private Turn turnToSwitch;
  private DriveStraight driveToSwitch;
  private DriveStraight driveBackwardsToScale;
  private Turn turnBackToScale;
  private GoToHeight goToGround;
  
  private Turn grabSecondCube;
  private Turn dropSecondCube;
  
  private DriveStraight getCloseToScale;
  private DriveStraight getToBox;
  private DriveStraight driveToBox;

  public HCStraightPath() {
    driveToScale = new LeftScaleFromLeft();
    turnToSwitch = new Turn(150, false, false);
    goToGround = new GoToHeight(0, 25, 15);
    getToBox = new DriveStraight(20, false, 0);
    getCloseToScale = new DriveStraight(20, false, 0);
    driveToBox = new DriveStraight(-20, false, 0);
    // driveToSwitch = new DriveStraight();
  }

  @Override
  public String getKey() {
    return "HCStraightPath";
  }

  @Override
  public void run() {
    driveToScale.run();
    runCommand(turnToSwitch, false);
    runCommand(goToGround, true);
    Robot.getDriveTrain().setStop();
  }
}
