package org.usfirst.frc.team2590.auto;

/**
 * A command that can be run
 * this is something like raising the elevator
 * or running the intake
 * @author Connor_Hofenbitzer
 *
 */
public interface NemesisRunnable {

    /**
     * runs a given command
     */
    public void run();

    /**
     * Checks if the given command is finished
     * @return : command is finished
     */
    public boolean isDone();


    /**
     * Gets the commands public key
     * @return : the commands key which is displayed on the dash board when run
     */
    public String getKey();
}
