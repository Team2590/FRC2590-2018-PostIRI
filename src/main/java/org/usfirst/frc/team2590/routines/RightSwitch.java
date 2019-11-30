package org.usfirst.frc.team2590.routines;

import org.usfirst.frc.team2590.auto.AutoRoutine;
import org.usfirst.frc.team2590.commands.DrivePath;
import org.usfirst.frc.team2590.commands.DriveStraight;
import org.usfirst.frc.team2590.commands.GoToHeight;
import org.usfirst.frc.team2590.commands.Turn;
import org.usfirst.frc.team2590.robot.Robot;

import edu.wpi.first.wpilibj.Timer;

public class RightSwitch extends AutoRoutine {

  private DrivePath driveToSwitch;
  private GoToHeight liftElevator;
  
  private DriveStraight driveParallel;
  private DriveStraight driveIntoSwitch;
  private Turn turnToSwitch;

  public RightSwitch() {
    liftElevator = new GoToHeight(30, 30 ,15);
  }

  @Override
  public String getKey() {
    // TODO Auto-generated method stub
    return "RIGHT SWITCH";
  }

  @Override
  public void run() {
    // TODO Auto-generated method stub
    
    driveParallel = new DriveStraight(105, true,0);
    turnToSwitch = new Turn(-90, false, false);
    driveIntoSwitch = new DriveStraight(5 , true, 90);
    
    if(getSwitchPos() == 'R') { //This is a Turn to Switch and Drop From the Right Side
      Robot.getElevator().moveSmooth(35, 40, 20);
      

      runCommand(driveParallel, false);
      runCommand(turnToSwitch, false);
      runCommand(driveIntoSwitch, false);
      Robot.getIntake().spit();
      Timer.delay(.25);
      Robot.getIntake().stop();
    } else { //This is Just a Drive Straight and Turn
      
      driveParallel = new DriveStraight(165, true , 0);
      Robot.getElevator().moveSmooth(35, 40, 20);

      runCommand(driveParallel, false);
      runCommand(turnToSwitch, false);
    }
    
  }


  @Override
  public void endAuto() {
    super.endAuto();
    driveToSwitch.cancel();
  }
}
