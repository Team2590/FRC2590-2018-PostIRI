package org.usfirst.frc.team2590.controls;

public class Controller {


  public void setSetpoint(double setpoint) {}
  public void setSetpoint(double currPos, double setpoint) {}
  public void changeGains(ControlPresets preset) {}

  public boolean isDone() {
    return false;
  }

  public double calculate(double currPos) {
    return 0.0;
  }
}
