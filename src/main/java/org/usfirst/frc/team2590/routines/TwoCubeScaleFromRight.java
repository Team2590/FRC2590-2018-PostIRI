package org.usfirst.frc.team2590.routines;

import org.usfirst.frc.team2590.auto.AutoRoutine;
import org.usfirst.frc.team2590.commands.DriveStraight;
import org.usfirst.frc.team2590.commands.DriveTillBlock;
import org.usfirst.frc.team2590.commands.GoToHeight;
import org.usfirst.frc.team2590.commands.Turn;
import org.usfirst.frc.team2590.robot.Robot;

import edu.wpi.first.wpilibj.Timer;

public class TwoCubeScaleFromRight extends AutoRoutine {

  //drops off at scale
  private Turn grabSecond;
  private Turn dropSecond;
  private Turn turnAway;
  
  private DriveStraight getClose;
  private DriveStraight crossLine;
  
  private RightScaleFromRight getToScale;
  private GoToHeight goToGround;
  private DriveStraight driveToBox;
  private DriveStraight getToBox;
  private DriveTillBlock getSecondB;
  private DriveStraight driveToSecondCube;
  private DriveStraight getBackToScale;
  private DriveStraight driveParallel;
  private DriveStraight driveIntoSwitch;
  private Turn turnToSwitch;
  
  public TwoCubeScaleFromRight() {
    getToBox = new DriveStraight(20, false, 0);
    getClose = new DriveStraight(20, false, 0);
    driveToBox = new DriveStraight(-20, false, 0);
    getBackToScale = new DriveStraight(-40, false, 0);
    turnAway = new Turn(-150, false, false);
    getSecondB = new DriveTillBlock(8, 2);
    driveToSecondCube = new DriveStraight(42, false, 0);
  }
  
  @Override
  public String getKey() {
    // TODO Auto-generated method stub
    return "TWO RIGHT SCALE";
  }

  @Override
  public void run() {
    getToScale = new RightScaleFromRight();
    grabSecond = new Turn(128*(getScalePos() == 'L'?1.29: -1.18), false, false); //127
    dropSecond = new Turn(20*(getScalePos() == 'L' ? 1.75 : -0.75) , false, false);
    goToGround = new GoToHeight(0, 25, 15);
    
   
//     getToScale.run();
//      Robot.getIntake().stop();
//     
    driveParallel = new DriveStraight(105, true,0);
    turnToSwitch = new Turn(-90, false, false);
    driveIntoSwitch = new DriveStraight(5 , true, 90);
    //Robot.getIntake().stop();
    //if(getScalePos() == 'L' ) { //what i did was basically removed the if statement since it doesn't need to be a l/r
    //case. getToScale is effectively running regardless if it's left or right so the if statement can be neglected
    //what's left to do is tune the values to ensure that the second option in the ternary is correct
      getToScale.run();
      Robot.getIntake().stop();
      driveToBox = new DriveStraight(-10, false, Robot.getDriveTrain().getGyro().pidGet());
      runCommand(driveToBox, false);
      runCommand(goToGround, true);
      runCommand(grabSecond, false);
      System.out.println("angle to cube " + Robot.getDriveTrain().getGyro().pidGet());
      runCommand(getToBox , false);
      runCommand(getSecondB, false);
      //runCommand(driveToSecondCube, false);
      //Robot.getIntake().succ();
      //runCommand(turnAway, false);
      getBackToScale = new DriveStraight(-30, false, 128*(getScalePos() == 'L'?1.75: -1.18));
      runCommand(getBackToScale, false);
      Robot.getIntake().autonSucc();
      //Robot.getElevator().moveSmooth(75, 30, 15);
      Robot.getElevator().moveSmooth(75, 80, 30);
      runCommand(dropSecond, false);
      getClose = new DriveStraight(7, false, 55);
      runCommand(getClose, false);
      Timer.delay(.5);
      Robot.getIntake().spit();
    //} else {

      
      
      /*
      if(getSwitchPos() == 'L') { //This is a Turn to Switch and Drop From the Left Side
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
      } */
      
      
      /*runCommand(grabSecond, false);
      getSecondB = new DriveTillBlock(19, 2);
      runCommand(getSecondB, false);
      Robot.getIntake().autonSucc();
      Robot.getElevator().moveSmooth(70, 80, 60);
      runCommand(getBackToScale, false);
      getBackToScale = new DriveStraight(-40, false, 128*(getScalePos() == 'L'?0.9375: -1.25));

      runCommand(dropSecond, false);
      Timer.delay(.1);
      Robot.getDriveTrain().setStop();
      Robot.getIntake().outakeAuto();  */
      
      Robot.getDriveTrain().setStop();
    //}
    
  }
}


