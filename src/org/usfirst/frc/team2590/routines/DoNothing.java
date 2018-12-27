package org.usfirst.frc.team2590.routines;

import org.usfirst.frc.team2590.auto.AutoRoutine;
import org.usfirst.frc.team2590.commands.Turn;
import org.usfirst.frc.team2590.robot.Robot;

public class DoNothing extends AutoRoutine {

  private Turn turn;
  
  @Override
  public String getKey() {
    return "DO NOTHING";
  }

  @Override
  public void run() {
    //this auto does nothing silly
    turn = new Turn(140, false, false);
    runCommand(turn, false);
    Robot.getDriveTrain().setStop();
    Robot.getElevator().moveSmooth(30, 10, 5);
    turn = new Turn(90, false, false);
    runCommand(turn, false);
    Robot.getElevator().moveSmooth(0, 10, 5);

  }

}
