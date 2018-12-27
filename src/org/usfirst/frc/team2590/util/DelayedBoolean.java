package org.usfirst.frc.team2590.util;

import edu.wpi.first.wpilibj.Timer;

public class DelayedBoolean {

  private double startTime;
  private double fuse;

  public DelayedBoolean(double fuseLength) {
    fuse = fuseLength*1000;
    startTime = Timer.getFPGATimestamp()*1000;
  }

  public boolean getDone() {
    return Timer.getFPGATimestamp()*1000 - startTime > fuse;
  }

}
