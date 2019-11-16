package org.usfirst.frc.team2590.commands;

import org.usfirst.frc.team2590.auto.NemesisRunnable;
import org.usfirst.frc.team2590.robot.Robot;

/**
 * Drives at a specified angle for a specified distance
 * @author Connor_Hofenbitzer
 *
 */
public class DriveStraight implements NemesisRunnable {

  private boolean quick;
  private boolean inited;
  private double driveStp;
  private double angleD;
  
  /**
   * Drives in a straight line, through motion profiling
   * @param stp : distance to travel in inches
   */
  public DriveStraight(double stp, boolean fast, double angle) {
    quick = fast;
    inited = false;
    driveStp = stp;
    angleD = angle;
  }

  @Override
  public void run() {

    //if we havent started driving straight, we should probably start doing that
    if(!inited) {
      Robot.getDriveTrain().driveStraight(driveStp, quick, angleD);
    }
    inited = true;
  }

  @Override
  public boolean isDone() {
    return inited && Robot.getDriveTrain().driveStraightDone();
  }

  @Override
  public String getKey() {
    return "STRAIGHT " + driveStp;
  }

}
