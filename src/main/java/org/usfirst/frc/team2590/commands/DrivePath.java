package org.usfirst.frc.team2590.commands;

import org.usfirst.frc.team2590.auto.NemesisRunnable;
import org.usfirst.frc.team2590.robot.Robot;
import org.usfirst.team2590.trajectory.DualTrajectory;
import org.usfirst.team2590.trajectory.TrajectoryFollow;
import org.usfirst.team2590.trajectory.TrajectoryReader;

/**
 * Follows a specified path
 * @author Connor_Hofenbitzer
 *
 */
public class DrivePath implements NemesisRunnable {

  TrajectoryFollow follower;
  DualTrajectory segs;
  boolean started = false;
  boolean reverseHeadings = false;
  private String profile;

  /**
   * Runs a path with a offset of 0 and reverse is false
   * @param profile : name of the profile
   */
  public DrivePath(String profile, boolean right) {
    //init the reader and feed it the profile name
    this(profile, false , 0, right);
  }

  /**
   * Runs a path
   * @param profile: path name
   * @param rev : if the heading is reversed
   * @param offset : angular offset in degrees
   * @param right : if the path is turning right (nearly all of our autos do this year) 
   */
  public DrivePath(String profile, boolean rev, double offset, boolean right ) {

    this.profile = profile;

    //init a new reader with the given offsets and settings
    TrajectoryReader reader = new TrajectoryReader(profile);
    
    reader.setReverseHeading(rev);
    reader.setHeadingOffset(offset);
    segs = reader.getSegmentArray();

    follower = new TrajectoryFollow(segs, right);
  }

  public void reversePath(boolean rev, boolean isRight) {
    follower.reverse(rev, isRight);
  }
  @Override
  public void run() {
    //if it hasnt started the path, start driving
    if(!started) {
      follower.start();
      Robot.getDriveTrain().runPath();
    }

    //then tell us that we have started
    started = true;
  }

  @Override
  public boolean isDone() {
    //check if we've finished driving
    return follower.isFinishedPath();
  }

  /**
   * stop the path by interrupting the thread that runs it
   */
  public void cancel() {
    follower.interrupt();
  }

  @Override
  public String getKey() {
    return "DRIVE PATH: " + profile;
  }
}
