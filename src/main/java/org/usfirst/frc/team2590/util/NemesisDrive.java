package org.usfirst.frc.team2590.util;

public class NemesisDrive {

  private double[] outputs;

  public NemesisDrive() {
    outputs = new double[2];
  }

  /**
   * Calculates the drive outputs
   * @param throttle : driving straight
   * @param turn     : turning power
   * @return : calculated arcade drive
   */
  public double[] calculate(double throttle, double turn) {

    double left = throttle + turn;
    double right = throttle - turn;

    outputs[0] = left;
    outputs[1] = right;

    return outputs;
  }
}
