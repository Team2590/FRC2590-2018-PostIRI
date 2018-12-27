package org.usfirst.frc.team2590.routines;

import org.usfirst.frc.team2590.auto.AutoRoutine;
import org.usfirst.frc.team2590.commands.DrivePath;
import org.usfirst.frc.team2590.commands.DriveStraight;
import org.usfirst.frc.team2590.commands.DriveTillBlock;
import org.usfirst.frc.team2590.commands.GoToHeight;
import org.usfirst.frc.team2590.commands.Turn;
import org.usfirst.frc.team2590.robot.Robot;

import edu.wpi.first.wpilibj.Timer;

public class ThreeBaby extends AutoRoutine {

    private GoToHeight goUp;
    private GoToHeight goUpA;
    private GoToHeight goDown;
    private GoToHeight goUpDropThree;

    private LeftScaleFromLeft screwIt;

    private DriveStraight backUp;
    private DriveStraight backScale;
    private DriveStraight driveIntoScale;

    //drives to grab another block
    private DriveTillBlock gitGud;
    private DriveTillBlock kobeGrab;

    private Turn turnAround;
    private Turn turnToThird;
    private Turn turnToScale;

    private DrivePath runnablePath;
       

    public ThreeBaby() {
      //elevator movement
      goUp = new GoToHeight(70 , 30 , 10);
      goUpA = new GoToHeight(40 , 90 , 60);
      goDown = new GoToHeight(0 , 80 , 40);
      goUpDropThree = new GoToHeight(80, 60, 60);

      //turn to the switch
      turnAround = new Turn(160, false, false);
      turnToThird = new Turn(-30, true, false);
      turnToScale = new Turn(-85 , true, false);

      //get over to pick up the next box
      backUp = new DriveStraight(40, false, 0 );
      backScale = new DriveStraight(-35, false, 0 );
      driveIntoScale = new DriveStraight(25, false, 0);

      gitGud = new DriveTillBlock(15, 5);
      kobeGrab = new DriveTillBlock(12, 5);

      screwIt = new LeftScaleFromLeft();

      //get to the scale
      runnablePath = new DrivePath("Scale", true, 0, true);
    }

    @Override
    public String getKey() {
      // TODO Auto-generated method stub
      return "THREE LEFT";
    }

    @Override
    public void run() {
      if(getScalePos() == 'L') {
        //drop off the first box
        Robot.getIntake().autonSucc();
        runCommand(goUp, true);
        runCommand(runnablePath, false);
        Robot.getDriveTrain().setStop();
        Robot.getIntake().spit();
        Timer.delay(.5);

        if(getScalePos() == getSwitchPos()) {
          //turn to the second box
          Robot.getIntake().stop();
          runCommand(turnAround, false);
          runCommand(goDown, true);

          //get the second box
          runCommand(backUp, false);
          runCommand(gitGud, false);
          Robot.getDriveTrain().setStop();

          //spit out the second box
          runCommand(goUpA, true);

          Timer.delay(.5);
          Robot.getIntake().spit();
          Timer.delay(.25);

          Robot.getIntake().stop();

          runCommand(goDown, true);
          runCommand(turnToThird, false);
          runCommand(kobeGrab, false);

          runCommand(backScale, false);
          runCommand(goUpDropThree,true);
          runCommand(turnToScale, false);
          runCommand(driveIntoScale, false);

          Robot.getIntake().outakeAuto();
        } else {

          turnAround = new Turn(-100, false, false);
          Robot.getIntake().stop();

          runCommand(turnAround, false);
          runCommand(goDown, true);

          backUp = new DriveStraight(80, false , 0);
          runCommand(backUp, false);

          runCommand(gitGud, false);
          Robot.getDriveTrain().setStop();

          //spit out the second box
          runCommand(goUpA, true);

          Timer.delay(.5);
          Robot.getIntake().spit();
          Timer.delay(.25);

          Robot.getIntake().stop();

        }
      } else {
        screwIt.run();
      }
  }

    @Override
    public void endAuto() {
      super.endAuto();
      runnablePath.cancel();
    }
  }


