package org.usfirst.frc.team2590.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoHandler {

  private AutoRoutine[] autoHolder;
  private AutoRoutine currentSelected;

  /**
   * Handles possible auto routines we could run
   * @param autos : all  of our autos
   */
  public AutoHandler(AutoRoutine... autos) {
    autoHolder = autos;
    currentSelected = null;
  }

  /**
   * Picks the auto you have chosen
   * @param autoKey : the name of the autonomous
   */
  public boolean pickAuto(String autoKey) {
    currentSelected = null;

    //this is probably really inefficient, try to find a dynamic way to do this next year
    for(AutoRoutine a : autoHolder) {
      
      //make it case insensitive
      if(a.getKey().toLowerCase().equals(autoKey.toLowerCase())) {
        currentSelected = a;
        return true;
      }
    }

    currentSelected = null;
    return false;
  }

  /**
   * Runs the chosen auto
   */
  public void runAuto() {
    
    //if there is an auto to run then run that auto
    if(currentSelected != null) {
      DriverStation.reportWarning("Starting auto " + currentSelected.getKey(), false);
      currentSelected.run();
    } else {
      
      //other wise make people know that they didnt actually choose an auto
      DriverStation.reportWarning("No auto chosen, running sit still", false);
    }
  }

  /**
   * Kills the autonomous mode
   */
  public void endAuto() {
    if( currentSelected != null ) {
      currentSelected.endAuto();
      currentSelected = null;
    }
  }



}
