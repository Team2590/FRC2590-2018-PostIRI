package org.usfirst.frc.team2590.util;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class NemesisLogger {
  
  private BufferedWriter matchFile;
  
  /**
   * Makes a new file within the robots classpath
   * @param fileName
   */
  public NemesisLogger(String fileName) {
    try {
      matchFile = new BufferedWriter(new FileWriter(fileName));
    } catch(Exception e) {
      e.printStackTrace();
    }
  }
  
  /**
   * Gives it a time stamp, match setting , etc etc
   * @param message : what to write to the file
   */
  public void pushMessage(String message) {
    
    //init some stuff
    double timestamp; 
    String currentMatchPoint;
    
    //gets the current point in the match
    if(DriverStation.getInstance().isDisabled()) {
      //get the time since the robot has been turned on if the robot is disabled
      timestamp = Timer.getFPGATimestamp();
      currentMatchPoint = "Disabled";
    } else {
      //get the time since the match has started if its enable
      timestamp = DriverStation.getInstance().getMatchTime();
      currentMatchPoint = DriverStation.getInstance().isAutonomous() ? "Auto" : "Teleop";
    }
    
    //actually write the message
    try {
      matchFile.write("Time : " + timestamp + 
          " \n Match mode: " + currentMatchPoint + 
          " \n Message: " + message + 
          " \n --------------------------------------");
    } catch (IOException e) {
      e.printStackTrace();
    }
  }
  
}
