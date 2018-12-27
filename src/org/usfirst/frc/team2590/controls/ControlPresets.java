package org.usfirst.frc.team2590.controls;

/**
 * Just a holster for control gains
 * @author Connor_Hofenbitzer
 *
 */
public class ControlPresets {

  private double kP;
  private double kF;
  private double kA;
  private double maxVel;
  private double maxAccel;

  public ControlPresets(double kP, double kF, double kA, double maxVel, double maxAccel) {
    this.kP = kP;
    this.kF = kF;
    this.kA = kA;
    this.maxVel = maxVel;
    this.maxAccel = maxAccel;
  }

  public double getkP() {
    return kP;
  }


  public double getkF() {
    return kF;
  }

  public double getkA() {
    return kA;
  }

  public double getmaxVel() {
    return maxVel;
  }

  public double getmaxAcc() {
    return maxAccel;
  }
}
