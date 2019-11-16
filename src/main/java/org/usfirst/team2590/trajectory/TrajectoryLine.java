package org.usfirst.team2590.trajectory;

import java.math.RoundingMode;
import java.text.DecimalFormat;

public class TrajectoryLine {

	public double acc;
	public double dist;
	public double time;
	public double[] initials;

	public TrajectoryLine(double[] initials, double acc, double dist) {
		super();
		this.acc = acc;
		this.dist = dist;
		this.initials = initials;

		this.time = solveQuadEq(0.5 * acc, initials[1], -dist);
	}

	public double[] getPointAtTime(double time){ //seconds
		double[] point = new double[3];

		if (time > this.time) {
      return null;
    }

		point[0] = .5 * this.acc * time * time + initials[1] * time +initials[0];
		point[1] = this.acc * time + initials[1];
		point[2] = this.acc;

		return point;
	}

	public double[] getNextInitials(){
		double[] point = new double[2];

		point[0] = dist + initials[0];
		point[1] = this.acc * this.time + initials[1];

		return point;
	}

	public double solveQuadEq(double a, double b, double c){
		if (a == 0) {
      return -c/b;
    }

		double temp1 = Math.sqrt(b * b - 4 * a * c);

		double root1 = (-b +  temp1) / (2*a) ;
		double root2 = (-b -  temp1) / (2*a) ;

		return (root1>0)?root1:root2;
	}

	@Override
  public String toString(){
		DecimalFormat df = new DecimalFormat("#.####");
		df.setRoundingMode(RoundingMode.HALF_UP);

		return "Dist: " + df.format(dist) + " Acc: " + df.format(acc) + " Time: " + df.format(time);
	}

}
