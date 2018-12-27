package org.usfirst.team2590.trajectory;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Scanner;

public class TrajectoryReader {

	String trajName;
	boolean reverseHeadings = false;
	double offsetHeading;

	public TrajectoryReader (String trajName){
		this.trajName = trajName;
	}

	public void setReverseHeading(boolean a){
		reverseHeadings = a;
	}

	public void setHeadingOffset(double a){
		offsetHeading = a;
	}
	public DualTrajectory getSegmentArray(){
		ArrayList<TrajectorySegment> ret = new ArrayList<TrajectorySegment>();
		DualTrajectory two = new DualTrajectory();

		try {
			Scanner scriptScanner = new Scanner(new FileReader(new File("/home/lvuser/paths/" + trajName + ".txt")));
			scriptScanner.useDelimiter("\\Z");
			String text = scriptScanner.next();
			scriptScanner.close();

			boolean beginSkip = true;
			for (String line : text.split("\n")){
				String[] strPieces = line.split(" ");
				double[] pieces = new double[strPieces.length];

				if ((line.isEmpty() || strPieces.length == 0 || !(strPieces[0].startsWith("0") || strPieces[0].startsWith("-0"))) && beginSkip == true) {
					continue;
				}
				beginSkip = false;

				for(int i = 0; i < strPieces.length; i++) {
          pieces[i] = Double.parseDouble(strPieces[i]);
        }

				TrajectorySegment s = new TrajectorySegment(pieces[5], pieces[0], pieces[1], pieces[2], (reverseHeadings?1:-1) * pieces[4] + offsetHeading);
				ret.add(s);

				//System.out.println("Setting Setpoint: " + s.pos);
			}

			two.left = new ArrayList<TrajectorySegment>(ret.subList(0, (ret.size()/2)));
			two.right = new ArrayList<TrajectorySegment>(ret.subList((ret.size()/2), ret.size()));

		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		return two;
	}
}
