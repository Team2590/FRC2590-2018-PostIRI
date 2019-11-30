package org.usfirst.team2590.trajectory;

import java.util.ArrayList;

public class DualTrajectory {

	ArrayList<TrajectorySegment> left;
	ArrayList<TrajectorySegment> right;

	public DualTrajectory(){
		left = new ArrayList<TrajectorySegment>();
		right = new ArrayList<TrajectorySegment>();
	}

	public DualTrajectory(ArrayList<TrajectorySegment> s){
		left = s;
		right = s;
	}

	public DualTrajectory(ArrayList<TrajectorySegment> r, ArrayList<TrajectorySegment> l){
		left = l;
		right = r;
	}

	public void reverseDirectionSide(String side){
		ArrayList<TrajectorySegment> chosen;

		if (side.equalsIgnoreCase("left")){
			chosen = left;
		} else if (side.equalsIgnoreCase("right")){
			chosen = right;
		} else {
			System.err.println("No correct side specified for left/right trajectory reversal");
			return;
		}

		for (int i = 0; i < chosen.size(); i++){
			chosen.get(i).acc *= -1;
			chosen.get(i).vel *= -1;
			chosen.get(i).pos *= -1;
		}
	}

}
