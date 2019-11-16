package org.usfirst.frc.team2590.looper;

@FunctionalInterface
public interface Loop {

  /**
   * Loop to run at a rate
   * @param delta : the time delta fed from the master loop
   */
	public void runLoop(double delta);

}
