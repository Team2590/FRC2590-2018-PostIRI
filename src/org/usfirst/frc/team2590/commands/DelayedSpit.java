package org.usfirst.frc.team2590.commands;

import org.usfirst.frc.team2590.auto.NemesisRunnable;
import org.usfirst.frc.team2590.robot.Robot;
import org.usfirst.frc.team2590.util.DelayedBoolean;

/**
 * Runs the intake after a specified amount of time
 * @author Connor_Hofenbitzer
 *
 */
public class DelayedSpit implements NemesisRunnable {

  private double length;
  private DelayedBoolean deployBool;

  /**
   * Moves the elevator to a desired height after a given time
   * @param delayLength : time to wait in seconds
   * @param height      : height to go to in inches
   */
  public DelayedSpit(double delayLength) {
    length = delayLength;
  }

  /**
   * thread that waits to run the command
   */
  private Runnable delayShot = () -> {

    //creates a delayed boolean
    deployBool = new DelayedBoolean(length);

    //wait until the boolean fires
    while(!deployBool.getDone()) { }

    //raise the elevator
    if(deployBool.getDone()) {
      Robot.getIntake().spit();
    }

  };



  @Override
  public void run() {

   //start the waiting thread
   Thread runner = new Thread(delayShot);
   runner.start();
  }

  @Override
  public boolean isDone() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public String getKey() {
    // TODO Auto-generated method stub
    return null;
  }

}
