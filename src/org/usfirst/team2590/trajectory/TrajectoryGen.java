package org.usfirst.team2590.trajectory;

import java.util.ArrayList;

public class TrajectoryGen {

	public static ArrayList<TrajectorySegment> generateStraightTrajectory(
			double distance, double maxVel, double maxAcc){
		ArrayList<TrajectorySegment> fin = new ArrayList<TrajectorySegment>();



		boolean isReverse = false;
		if (distance < 0){
			isReverse = true;
			distance *= -1;
		}

		ArrayList<TrajectoryLine> profile = calculateTrapizoid(distance, maxAcc, maxVel);

		//System.out.println(profile);

		double interval = 0.01; //seconds

		for(TrajectoryLine p : profile){
			for(double foo = 0.0; foo < p.time; foo += interval){

				double[] point = p.getPointAtTime(foo);
				TrajectorySegment segment = new TrajectorySegment(interval,
						point[0] * (isReverse?-1:1),
						point[1] * (isReverse?-1:1),
						point[2] * (isReverse?-1:1),
						0.0);

				fin.add(segment);
			}
		}

		//long start = System.currentTimeMillis();
		//System.out.println("Time: " + (System.currentTimeMillis() - start));

		return fin;
	}

	private static ArrayList<TrajectoryLine> calculateTrapizoid(double distance, double maxAcc, double maxVel){
		double halfDist = distance / 2.0;

		// 1/2 a t^2 = d --- t = vf / a
		double maxAccDist = (.5) * (maxVel * maxVel) / maxAcc;

		ArrayList<TrajectoryLine> temp = new ArrayList<TrajectoryLine>();
		if(maxAccDist > halfDist){
			temp.add(new TrajectoryLine(new double[]{0,0}, maxAcc, halfDist));
			temp.add(new TrajectoryLine(temp.get(temp.size() - 1).getNextInitials(), -maxAcc, halfDist));

			return temp;
		} else {

			temp.add(new TrajectoryLine(new double[]{0,0}, maxAcc, maxAccDist));
			temp.add(new TrajectoryLine(temp.get(temp.size() - 1).getNextInitials(), 0, (2.0 * (halfDist - maxAccDist))));
			temp.add(new TrajectoryLine(temp.get(temp.size() - 1).getNextInitials(), -maxAcc, maxAccDist));

			return temp;
		}
	}
}
