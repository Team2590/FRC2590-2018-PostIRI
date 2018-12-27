package org.usfirst.frc.team2590.auto;

import org.usfirst.frc.team2590.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Base auto stuff
 * @author Connor_Hofenbitzer
 *
 */
public abstract class AutoRoutine {

  private boolean running = true;
  public abstract String getKey();
  public abstract void run();


  /**
   * Runs the given command
   * @param runnable : command to run
   * @param isParallel : if it runs in Parallel or sequential. Parallel commands don't hold up the queue
   * Sequential commands wait for the command to be finished
   */
  public void runCommand(NemesisRunnable runnable, boolean isParallel) {
    //if the command is supposed to be running
    if(running) {

      //sequential case
      if(!isParallel) {

        //if were not in autonomous, then don't autonomous
        while(running && DriverStation.getInstance().isAutonomous())  {
          //forcing an end condition
          if(runnable.isDone()) {
            return;
          }
          
          //cycle the command
          runnable.run();
          try {
            Thread.sleep(50);
          } catch (Exception e) {
            e.printStackTrace();
          }

        }

      //Parallel case
      } else {
        runnable.run();
      }
    }

  }

  /**
   * Finishes auto, gets all subsystems ready for teleop
   */
  public void endAuto( ) {
    running = false;
    Robot.getIntake().stop();
    Robot.getDriveTrain().setStop();
    Robot.getElevator().stopElevator();
  }



  /**
   * Gets your switch
   * @return L for left and R for right
   */
  public char getSwitchPos() {
    return DriverStation.getInstance().getGameSpecificMessage().charAt(0);
  }

  /**
   * Gets your scale
   * @return L for left and R for right
   */
  public char getScalePos() {
    return DriverStation.getInstance().getGameSpecificMessage().charAt(1);
  }

}
