package org.usfirst.frc.team2590.util;

import java.util.LinkedList;
import java.util.Queue;

public class AveragingQueue {

  private int historyLength;
  private Queue<Double> history;

  private double average;
  private double runningTotal;

  /**
   * A collection algorithm which constantly averages
   * @param historyL : length of the history
   */
  public AveragingQueue(int historyL) {
    average = 0.0;
    runningTotal = 0;

    historyLength = historyL;
    history = new LinkedList<Double>();
  }

  /**
   * Adds a number to the queue and recomputes the average
   * @param number : number to add to the queue
   */
  public void update(double number) {
    runningTotal += number;
    history.add(number);
    if(history.size() > historyLength) {
      runningTotal -= history.remove();
    }

    average = runningTotal / history.size();
  }

  /**
   * Gets the average of all numbers in the queue
   * @return : the average
   */
  public double getAverage() {
    return average;
  }

  /**
   * Gets the size of the averaging array
   * @return : the size of the averaging array
   */
  public int getCurrentSize() {
    return history.size();
  }
}
