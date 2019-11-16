package org.usfirst.team2590.trajectory;

import java.math.RoundingMode;
import java.text.DecimalFormat;

public class TrajectorySegment {
	double dt, pos, vel, acc, heading;

	public TrajectorySegment(double dt, double pos, double vel, double acc, double heading) {
		super();
		this.dt = dt;
		this.pos = pos;
		this.vel = vel;
		this.acc = acc;
		this.heading = heading;
	}

	public double getTime() {
		return dt;
	}

	public double getPos() {
		return pos;
	}

	public double getVel() {
		return vel;
	}

	public double getAcc() {
		return acc;
	}

	public double getHeading() {
		return heading;
	}

	@Override
  public String toString(){
		DecimalFormat df = new DecimalFormat("#.###");
		df.setRoundingMode(RoundingMode.HALF_UP);

		return "\nDt: " + df.format(dt) +
				" Pos: " + df.format(pos) +
				" Vel: " + df.format(vel) +
				" Acc: " + df.format(acc) +
				" Heading: " + df.format(heading);
	}


}
