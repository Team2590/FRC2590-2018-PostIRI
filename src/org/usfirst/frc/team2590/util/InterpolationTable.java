package org.usfirst.frc.team2590.util;

public class InterpolationTable {

  private double[] xPoints;
  private double[] yPoints;
  

  public void addXPoints(double... x) {
    xPoints = x;
  }
  
  public void addYPoints(double... y) {
    yPoints = y;
  }
  
  public double findYatX(double x) {
   if(x < xPoints[0] || x > xPoints[xPoints.length]) {
     throw new IndexOutOfBoundsException("please make sure x is greater than the first "
         + "index and less than the last index x recieved: " + x );
   }
   
   if((xPoints == null) || (yPoints == null) || (xPoints.length != yPoints.length)) {
     throw new NullPointerException("Please initialize two arrays of equal length");
   }
    
    return 0.0;
  }
  
}
