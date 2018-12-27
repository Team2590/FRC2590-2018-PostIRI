package org.usfirst.frc.team2590.util;

import edu.wpi.first.wpilibj.Encoder;

public class NemesisEncoder extends Encoder {

  private double offSet;
  public NemesisEncoder(int  sourceA, int sourceB) {
    super(sourceA, sourceB);    // TODO Auto-generated constructor stub
    offSet = 0;
  }

  public void setDistance(double offset) {
    offSet = offset - super.getDistance();
  }

  @Override
  public double getDistance() {
    return super.getDistance() + offSet;
  }

  @Override
  public void reset() {
    super.reset();
    offSet = 0;
  }

}
