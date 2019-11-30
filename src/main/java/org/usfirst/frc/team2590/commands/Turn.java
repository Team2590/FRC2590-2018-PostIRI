package org.usfirst.frc.team2590.commands;

import org.usfirst.frc.team2590.auto.NemesisRunnable;
import org.usfirst.frc.team2590.robot.Robot;

/**
 * Turns to a given angle
 * @author Connor_Hofenbitzer
 *
 */
public class Turn implements NemesisRunnable {

  private boolean quick;
  private boolean reset;
  private double setpoint;
  private boolean started;
  
  public Turn(double setpoint, boolean reset, boolean fast) {
    this.quick = fast;
    this.reset = reset;
    this.started = false;
    this.setpoint = setpoint;
    System.out.println("stp " + setpoint);
  }
  @Override
  public void run() {
    if(!started) {
      Robot.getDriveTrain().turn(setpoint, reset, quick);
      started = true;
    }
  }

  @Override
  public boolean isDone() {
    System.out.println("is done " + (Robot.getDriveTrain().turnDone() && started));
    return Robot.getDriveTrain().turnDone() && started;
  }

  @Override
  public String getKey() {
    return null;
  }

}
