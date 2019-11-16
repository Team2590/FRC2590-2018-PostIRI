package org.usfirst.frc.team2590.looper;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

/**
 * The handler for update loops
 * @author Connor_Hofenbitzer
 */
public class Looper {

  private double delayT = 0;
  private Notifier notfifier;
  private double lastTime = 0;
  private double currentTime = 0;
  private boolean running = false;
  private ArrayList<Loop> loopArray;


  /**
   * The runnable looper
   */
  private Runnable masterLoop = () -> {
    if(running) {
        //periodically update the loops
        currentTime = Timer.getFPGATimestamp();
        for(Loop l : loopArray) {
          l.runLoop(currentTime-lastTime);
        }
        lastTime = currentTime;
      }

  };

  /**
   * Master loop class, handles all other loops
   * @param delayTime : time to wait in between cycles
   */
  public Looper(double delayTime) {
    delayT = delayTime;
    loopArray = new ArrayList<Loop>();
    notfifier = new Notifier(masterLoop);
  }

  /**
   * add a new loop to the arraylist
   * @param loop : its a loop
   */
  public void register(Loop loop) {
    loopArray.add(loop);
  }

  /**
   * Starts all the loops
   */
  public void startLoops() {
    System.out.println("trying to loops");
    if(!running) {
      System.out.println("starting loops");
      running = true;
      notfifier.startPeriodic(delayT);
    }
  }

  /**
   * Ends the loops
   */
  public void onEnd() {
    running = false;
  }
}


