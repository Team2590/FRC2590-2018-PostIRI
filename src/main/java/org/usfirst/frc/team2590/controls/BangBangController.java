package org.usfirst.frc.team2590.controls;

public class BangBangController extends Controller {

  private boolean done;
  private double cycles;
  private double setpoint;
  private double tolerance;
  private double controlSpeed;

  /**
   * New bang bang controller
   * @param controlSpeed: speed for the system to run at
   */
  public BangBangController(double controlSpeed, double tol) {
    this.cycles = 0;
    this.done = false;
    this.setpoint = 0;
    this.tolerance = tol;
    this.controlSpeed = controlSpeed;
  }


  @Override
  public void setSetpoint(double setpoint) {
    this.cycles = 0;
    this.done = false;
    this.setpoint = setpoint;
  }

  @Override
  public double calculate(double currPos) {
    done = Math.abs(setpoint - currPos) < tolerance;
    if(done) {
      cycles += 1;
      return 0.0;
    }
    cycles = 0;
    return (setpoint - currPos) > 0 ? controlSpeed : -controlSpeed;
  }

  @Override
  public boolean isDone() {
    return done && cycles > 10;
  }
}
