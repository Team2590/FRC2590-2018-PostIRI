package org.usfirst.frc.team2590.commands;

import org.usfirst.frc.team2590.auto.NemesisRunnable;
import org.usfirst.frc.team2590.robot.Robot;

import edu.wpi.first.wpilibj.Timer;


/**
 * Drives at a specified speed until the robot has intaked (intooken?) a box
 * @author Connor_Hofenbitzer
 *
 */
public class DriveTillBlock implements NemesisRunnable {

  private double speed;
  private double timeout;
  private boolean engaged;
  private double startTime;
  
  
  public DriveTillBlock(double speed, double timeout) {
    this.startTime = 0;
    this.speed = speed;
    this.engaged = false;
    this.timeout = timeout;
  }

  @Override
  public void run() {
    //start the time out
    if(!engaged) {
      startTime = Timer.getFPGATimestamp();
      engaged = true;
    }

    if(!Robot.getIntake().checkBox()) {
      Robot.getDriveTrain().setVelocity(speed);
      Robot.getIntake().succ();
    } else {
      Robot.getIntake().stop();
      Robot.getDriveTrain().setStop();
    }
  }

  @Override
  public boolean isDone() {
    return Robot.getIntake().checkBox() || (Math.abs(Timer.getFPGATimestamp() - startTime ) > timeout && engaged);
  }

  @Override
  public String getKey() {
    return "GRAB N' GO AT SPEED " + speed;
  }

}
