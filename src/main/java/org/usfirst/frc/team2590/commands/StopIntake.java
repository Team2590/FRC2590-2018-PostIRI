package org.usfirst.frc.team2590.commands;

import org.usfirst.frc.team2590.auto.NemesisRunnable;
import org.usfirst.frc.team2590.robot.Robot;

/**
 * Stops running the intake
 * @author Connor_Hofenbitzer
 *
 */
public class StopIntake implements NemesisRunnable {

  @Override
  public void run() {
    Robot.getIntake().stop();
  }

  @Override
  public boolean isDone() {
    // TODO Auto-generated method stub
    return true;
  }

  @Override
  public String getKey() {
    // TODO A uto-generated method stub
    return null;
  }

}
