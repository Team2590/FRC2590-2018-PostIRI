package org.usfirst.frc.team2590.controls;

/**
 * Feed forward / feedback controller based around velocity
 * @author Connor_Hofenbitzer
 *
 */
public class VelocityController extends Controller {

  private double kP;
  private double kF;
  private double setpoint;

  public VelocityController(double kP, double kF) {
    this.kP = kP;
    this.kF = kF;
    this.setpoint = 0;
  }

  @Override
  public void setSetpoint(double newVel) {
    setpoint = newVel;
  }

  @Override
  public double calculate(double currvel) {
    return kF*setpoint + (currvel - setpoint)*kP;
  }
}
