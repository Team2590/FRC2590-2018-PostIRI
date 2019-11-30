package org.usfirst.frc.team2590.commands;

import org.usfirst.frc.team2590.auto.NemesisRunnable;
import org.usfirst.frc.team2590.robot.Robot;

/**
 * Spends the elevator to a given height
 * @author Connor_Hofenbitzer
 *
 */
public class GoToHeight implements NemesisRunnable {

  private double height;
  private double velocity;
  private double acceleration;


  public GoToHeight(double height, double velocity, double acceleration) {
    this.height = height;
    this.velocity = velocity;
    this.acceleration = acceleration;
  }

  @Override
  public void run() {
    Robot.getElevator().moveSmooth(height, velocity, acceleration);
  }

  @Override
  public boolean isDone() {
    return Robot.getElevator().isDone();
  }

  @Override
  public String getKey() {
    return "GOING TO HEIGHT " + height;
  }

}
