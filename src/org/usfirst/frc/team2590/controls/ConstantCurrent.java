package org.usfirst.frc.team2590.controls;

public class ConstantCurrent extends Controller {

  private double volts;

  @Override
  public void setSetpoint(double setpoint) {
    volts = setpoint;
  }

  @Override
  public double calculate(double control) {
    return volts;
  }
}
