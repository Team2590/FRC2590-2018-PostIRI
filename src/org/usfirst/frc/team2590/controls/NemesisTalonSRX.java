package org.usfirst.frc.team2590.controls;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * Restructured talon SRX
 * @author Griffin
 *
 */
public class NemesisTalonSRX extends TalonSRX implements SpeedController {
  
  private ControlMode ctrmode = ControlMode.PercentOutput;

  public NemesisTalonSRX(int deviceNumber) {
    super(deviceNumber);
  }

  public void setControlMode(ControlMode c) {
    ctrmode = c;
  }
  
  @Override
  public void set(double speed) {
    this.set(ctrmode, speed);    
  }

  @Override
  public double get() {
    return this.getMotorOutputPercent();
  }

  @Override
  public void disable() {
    this.disable();
  }

  @Override
  public void stopMotor() {
    this.stopMotor();
  }

  @Override
  public void pidWrite(double output) {
    this.set(output);
  }

}
