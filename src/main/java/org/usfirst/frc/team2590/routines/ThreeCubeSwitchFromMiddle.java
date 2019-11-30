package org.usfirst.frc.team2590.routines;

import org.usfirst.frc.team2590.auto.AutoRoutine;
import org.usfirst.frc.team2590.commands.DriveStraight;
import org.usfirst.frc.team2590.commands.DriveTillBlock;
import org.usfirst.frc.team2590.commands.GoToHeight;
import org.usfirst.frc.team2590.commands.Turn;
import org.usfirst.frc.team2590.robot.Robot;

import edu.wpi.first.wpilibj.Timer;

public class ThreeCubeSwitchFromMiddle extends AutoRoutine{
  
  private TwoCubeSwitchFromMiddle dropFirstAndSecondCube;
  private Turn pickUpThird;
  private Turn turnToSwitchToDrop;
  private DriveStraight backOutToGetThirdCube;
  private GoToHeight dropElevator;
  private DriveTillBlock grabThirdCube;
  private GoToHeight raiseElevator;
  private DriveStraight driveToSwitch;
  private DriveStraight backOutToPlaceThirdCube;
  

  @Override
  public String getKey() {
    // TODO Auto-generated method stub
    return "THREE MIDDLE SWITCH";
  }

  @Override
  public void run() {
    dropFirstAndSecondCube = new TwoCubeSwitchFromMiddle();
    backOutToGetThirdCube = new DriveStraight(-40, false, 0); //40
    pickUpThird  = new Turn(getSwitchPos() == 'R' ? -45 : 45, false,false);  //37
    dropElevator  = new GoToHeight(1, 45, 30);
    grabThirdCube = new DriveTillBlock(20, 3);
    
    backOutToPlaceThirdCube = new DriveStraight(-20, false, getSwitchPos() == 'R' ? 0 : -0);
    turnToSwitchToDrop  = new Turn(getSwitchPos() == 'R' ? 0 : -0, false,false); 
    raiseElevator = new GoToHeight(40, 60, 40);
    driveToSwitch = new DriveStraight(35, false, 0);

    dropFirstAndSecondCube.run();
    runCommand(backOutToGetThirdCube, false);
    runCommand(pickUpThird, false);
    Robot.getDriveTrain().setStop();
    runCommand(dropElevator, true);
    runCommand(grabThirdCube, false);
    
    //Include for Placing Third Cube
    /*
    runCommand(backOutToPlaceThirdCube, false);
    runCommand(turnToSwitchToDrop, false);
    Robot.getDriveTrain().setStop();
    runCommand(raiseElevator, true);
    runCommand(driveToSwitch, false);
    Timer.delay(.45);
    Robot.getIntake().spit();
    */
    
  }

}
