package org.usfirst.frc.team2590.routines;

import org.usfirst.frc.team2590.auto.AutoRoutine;
import org.usfirst.frc.team2590.commands.DriveStraight;
import org.usfirst.frc.team2590.commands.DriveTillBlock;
import org.usfirst.frc.team2590.commands.GoToHeight;
import org.usfirst.frc.team2590.commands.Turn;
import org.usfirst.frc.team2590.robot.Robot;

import edu.wpi.first.wpilibj.Timer;

public class TwoCubeSwitchFromMiddle extends AutoRoutine{
  
  private Turn pickUpNext;
  private Turn turnToDrop;
  private DriveStraight backOutToTurn;
  private GoToHeight dropElevator;
  private DriveTillBlock grabbyBoi;
  private MiddleSwitch getFirstOff;
  private GoToHeight raiseElevator;
  private DriveStraight driveToSwitch;
  private DriveStraight backOut;

  @Override
  public String getKey() {
    // TODO Auto-generated method stub
    return "TWO MIDDLE SWITCH";
  }

  @Override
  public void run() {
    getFirstOff = new MiddleSwitch();
    backOut = new DriveStraight(-20, false, getSwitchPos() == 'R' ? 0 : -0);
    driveToSwitch = new DriveStraight(35, false, 0);
    backOutToTurn = new DriveStraight(-40, false, 0); //40
    
    grabbyBoi = new DriveTillBlock(20, 3);
    dropElevator  = new GoToHeight(1, 60, 40);
    raiseElevator = new GoToHeight(40, 60, 40);
    turnToDrop  = new Turn(getSwitchPos() == 'R' ? 0 : -0, false,false); 
    pickUpNext  = new Turn(getSwitchPos() == 'R' ? -45 : 45, false,false);  //37
    
    getFirstOff.run();
    runCommand(backOutToTurn, false);
    runCommand(pickUpNext, false);
    Robot.getDriveTrain().setStop();
    runCommand(dropElevator, true);
    runCommand(grabbyBoi, false);
    runCommand(backOut, false);
    runCommand(turnToDrop, false);
    Robot.getDriveTrain().setStop();
    runCommand(raiseElevator, true);
    runCommand(driveToSwitch, false);
    Timer.delay(.45);
    Robot.getIntake().spit();
    
  }

}
