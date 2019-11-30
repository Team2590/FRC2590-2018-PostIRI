package org.usfirst.team2590.trajectory;

import org.usfirst.frc.team2590.robot.Robot;
import org.usfirst.team2590.settings.DriveTrainSettings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TrajectoryFollow extends Thread implements DriveTrainSettings{

  DualTrajectory traj;
  boolean done = false;
  public TrajectoryPIDController leftController, rightController;

  public TrajectoryFollow(DualTrajectory traj, boolean goingRight) {
    this.traj = traj;

    if(goingRight) {

      rightController = new TrajectoryPIDController(motionProfilekP, 0, 0,
          motionProfilekF, motionProfilekA, motionProfilekH, motionProfilekR,
          Robot.getDriveTrain().getLeftEncoder(), Robot.getDriveTrain().getLeftDrive(), Robot.getDriveTrain().getGyro());

      leftController = new TrajectoryPIDController(motionProfilekP, 0, 0,
          motionProfilekF, motionProfilekA, motionProfilekH, motionProfilekR,
          Robot.getDriveTrain().getRightEncoder(), Robot.getDriveTrain().getRightDrive(), Robot.getDriveTrain().getGyro());

    } else {
       leftController= new TrajectoryPIDController(motionProfilekP, 0, 0,
          motionProfilekF, motionProfilekA, motionProfilekH, motionProfilekR,
          Robot.getDriveTrain().getLeftEncoder(), Robot.getDriveTrain().getLeftDrive(), Robot.getDriveTrain().getGyro());

      rightController = new TrajectoryPIDController(motionProfilekP, 0, 0,
          motionProfilekF, motionProfilekA, motionProfilekH, motionProfilekR,
          Robot.getDriveTrain().getRightEncoder(), Robot.getDriveTrain().getRightDrive(), Robot.getDriveTrain().getGyro());
    }

    rightController.setTurnDirection(true);
    
    //leftController.setTurnDirection(true);
  }

  public void reverse(boolean reverse, boolean isRight) {
    if(!isRight) {

      rightController = new TrajectoryPIDController(-motionProfilekP, 0, 0,
          -motionProfilekF, -motionProfilekA, -motionProfilekH, -motionProfilekR,
          Robot.getDriveTrain().getLeftEncoder(), Robot.getDriveTrain().getLeftDrive(), Robot.getDriveTrain().getGyro());

      leftController = new TrajectoryPIDController(-motionProfilekP, 0, 0,
          -motionProfilekF, -motionProfilekA, -motionProfilekH, -motionProfilekR,
          Robot.getDriveTrain().getRightEncoder(), Robot.getDriveTrain().getRightDrive(), Robot.getDriveTrain().getGyro());

    } else {
       leftController= new TrajectoryPIDController(-motionProfilekP, 0, 0,
          -motionProfilekF, -motionProfilekA, -motionProfilekH, -motionProfilekR,
          Robot.getDriveTrain().getLeftEncoder(), Robot.getDriveTrain().getLeftDrive(), Robot.getDriveTrain().getGyro());

      rightController = new TrajectoryPIDController(-motionProfilekP, 0, 0,
          -motionProfilekF, -motionProfilekA, -motionProfilekH, -motionProfilekR,
          Robot.getDriveTrain().getRightEncoder(), Robot.getDriveTrain().getRightDrive(), Robot.getDriveTrain().getGyro());
    }

    rightController.setTurnDirection(!reverse);
    leftController.setTurnDirection(reverse);
  }
  @Override
  public void run(){

    long start = System.currentTimeMillis();

    leftController.reset();
    leftController.enable();
    rightController.reset();
    rightController.enable();

    Robot.getDriveTrain().resetAllSensors();

    System.out.println("Running traj");
    int segment = 0;
    while(segment < traj.left.size()) {
      //System.out.println("in traj loop");
      long current = System.currentTimeMillis();
      long dif = current - start;
      
      TrajectorySegment leftSegment = traj.left.get(segment);
      TrajectorySegment rightSegment = traj.right.get(segment);
      
      //desired turn rate pulls from the path segments (independent from gyro reading)
      //turn rate is the difference between each side's velocity, divided by the width of the wheelbase
      double desiredTurnRate = (leftSegment.vel - rightSegment.vel)/WheelBaseWidth;

      leftController.setSetpoint(leftSegment, desiredTurnRate);
      rightController.setSetpoint(rightSegment, desiredTurnRate);

      SmartDashboard.putNumber("Left Trajectory Pos: ", traj.left.get(segment).pos);
      SmartDashboard.putNumber("Right Trajectory Pos: ", traj.right.get(segment).pos);


      segment = (int) Math.round(dif / ((traj.left.get(0).dt) * 1000));
      try {
        Thread.sleep(4);
      } catch (InterruptedException e) {
        e.printStackTrace();
        break;
      }
    }

    done = true;
    rightController.disable();
    leftController.disable();
    Robot.getDriveTrain().teleopDrive(0, 0);
  }

  public boolean isFinishedPath(){
    return done;
  }

}
