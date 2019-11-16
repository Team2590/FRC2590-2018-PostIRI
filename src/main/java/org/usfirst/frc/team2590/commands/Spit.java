package org.usfirst.frc.team2590.commands;

import org.usfirst.frc.team2590.auto.NemesisRunnable;
import org.usfirst.frc.team2590.robot.Robot;

/**
 * just outtakes
 * @author Connor_Hofenbitzer
 *
 */
public class Spit implements NemesisRunnable {


  @Override
  public void run() {
    Robot.getIntake().spit();
  }

  @Override
  public boolean isDone() {
    // TODO Auto-generated method stub
    return true;
  }

  @Override
  public String getKey() {
    // TODO Auto-generated method stub
    return null;
  }

}
