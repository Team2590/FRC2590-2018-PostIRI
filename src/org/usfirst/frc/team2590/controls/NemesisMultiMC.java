package org.usfirst.frc.team2590.controls;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * Couples motor controllers together for
 * each side for the drivetrain
 * @author Griffin
 *
 */
public class NemesisMultiMC implements SpeedController {
  
  private SpeedController[] scs;
  private double output = 0.0;
  private boolean allInverted = false;
  private boolean[] scInverted;
  
  public NemesisMultiMC (SpeedController ... scs) {
    this.scs = scs;
    this.scInverted = new boolean[scs.length];
  }

  @Override
  public void pidWrite(double output) {
    this.set(output);
    this.output = output;
  }

  @Override
  public void set(double speed) {
    for (int i = 0; i < scs.length; i++) {
      scs[i].set(speed * (scInverted[i] ? -1 : 1));
    }
  }

  @Override
  public double get() {
    return this.output;
  }
  
  /**
   * Inverts the motor controller locally within this object, does not
   * call the inversion method of the speed controller
   * 
   * @param index Index of SpeedController to invert
   * @param inverted Status of the inversion
   */
  public void setIndividualInverted(int index, boolean inverted) {
    scInverted[index] = inverted;
  }

  /**
   * Get the local inversion status of the motor controller
   * 
   * @param index Index of SpeedController
   * @return Status of the inversion
   */
  public boolean getIndividualInverted(int index) {
    return scInverted[index];
  }
  
  @Override
  public void setInverted(boolean isInverted) {
    allInverted = isInverted;
  }

  @Override
  public boolean getInverted() {
    return allInverted;
  }

  @Override
  public void disable() {
    for (SpeedController s : scs) {
      s.disable();
    }
  }

  @Override
  public void stopMotor() {
    for (SpeedController s: scs) {
      s.stopMotor();
    }
  }


}
