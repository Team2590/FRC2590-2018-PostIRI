package org.usfirst.frc.team2590.robot;

import org.usfirst.frc.team2590.auto.AutoHandler;
import org.usfirst.frc.team2590.looper.Looper;
import org.usfirst.frc.team2590.routines.DoNothing;
import org.usfirst.frc.team2590.routines.DriveFive;
import org.usfirst.frc.team2590.routines.HCStraightPath;
import org.usfirst.frc.team2590.routines.LeftScaleFromLeft;
import org.usfirst.frc.team2590.routines.RightScaleFromRight;
import org.usfirst.frc.team2590.routines.MiddleSwitch;
import org.usfirst.frc.team2590.routines.MiddleSwitchFast;
import org.usfirst.frc.team2590.routines.LeftSwitch;
import org.usfirst.frc.team2590.routines.RightSwitch;
import org.usfirst.frc.team2590.routines.TwoCubeSwitchFromMiddle;
import org.usfirst.frc.team2590.routines.ThreeCubeSwitchFromMiddle;
import org.usfirst.frc.team2590.routines.NullCube;
import org.usfirst.frc.team2590.routines.NullCubeRight;
import org.usfirst.frc.team2590.routines.ThreeBaby;
import org.usfirst.frc.team2590.routines.TwoCubeScaleFromLeft;
import org.usfirst.frc.team2590.routines.TwoCubeScaleFromRight;
import org.usfirst.frc.team2590.subsystems.Climber;
import org.usfirst.frc.team2590.subsystems.Drivetrain;
import org.usfirst.frc.team2590.subsystems.Elevator;
import org.usfirst.frc.team2590.subsystems.Intake;
import org.usfirst.frc.team2590.usbVision.CameraBoi;
import org.usfirst.frc.team2590.usbVision.CameraHandler;
import org.usfirst.frc.team2590.usbVision.CameraPresets;
import org.usfirst.frc.team2590.util.NemesisJoystick;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The main class for FRC 2590s 2018 Robot Code
 * @authors Connor_Hofenbitzer, Harsh Padhye, Chinmay Savanur
 */
public class Robot extends IterativeRobot {

  //misc

	private static Intake intake;
	private static Climber climber;
	private static Elevator elevator;
	private static Drivetrain driveTrain;
	
	private Looper enabledLooper;
	private AutoHandler autonHandler; //$name this 'auton' for clarity
	 
	private NemesisJoystick leftJ;
	private NemesisJoystick rightJ;
	private NemesisJoystick operatorJ;

	private boolean init;
	private boolean started;
	
	private CameraHandler frontCamera;
	private static CameraBoi cameraBoyo;
	
	private Compressor compres;
	@Override
	public void robotInit() {
	  
	  init = true;
	  started = false;
	  //init misc stuffs
		enabledLooper = new Looper(0.01); //$have a constant for this variable

		//init stuff that has to be init
		intake = Intake.getIntakeInstance();
    climber = Climber.getClimberInstance();
		elevator = Elevator.getElevatorInstance();
		driveTrain = Drivetrain.getDriveInstance();

		//register loopers
    enabledLooper.register(driveTrain::loop);
    enabledLooper.register(intake::update);
    enabledLooper.register(climber::update);
    enabledLooper.register(elevator::update);

    autonHandler = new AutoHandler(new DoNothing(), new MiddleSwitch(), new LeftSwitch(), new RightSwitch(), new LeftScaleFromLeft(), 
                                   new ThreeBaby(), new TwoCubeScaleFromLeft(), new DriveFive(), new TwoCubeSwitchFromMiddle(), new MiddleSwitchFast(), 
                                   new TwoCubeScaleFromRight(), new NullCube(), new NullCubeRight(), new HCStraightPath(), new ThreeCubeSwitchFromMiddle(), 
                                   new RightScaleFromRight());

    leftJ = new NemesisJoystick(0 , 0.1 , 0.1);
    rightJ = new NemesisJoystick(1 , 0.25 , 0.25);
    operatorJ = new NemesisJoystick(2 , 0.25 , 0.25);

    frontCamera = new CameraHandler(0, new CameraPresets(30, 320, 240, 20, 20, 20));

    compres = new Compressor();
    compres.start();
    
    cameraBoyo = CameraBoi.getVisionInstance();
	}

	@Override
  public void disabledPeriodic() {
	  
	  //string picks auto
	  SmartDashboard.putBoolean("DB/LED 0", autonHandler.pickAuto(SmartDashboard.getString("DB/String 0", "DO NOTHING")));

	  //sets all of the drive motors to coast
	  driveTrain.setAllMode(false);
    
	  //resets all of the things
	  intake.stop();
    driveTrain.setStop();
    
    //continuosoly resets the compressor
    compres.clearAllPCMStickyFaults();
    
	}

  @Override
  public void autonomousInit() {
    //sets the teleop init boolean
    init = false;
    
    driveTrain.resetAllSensors();
    
    //starts auto and the robot
    enabledLooper.startLoops();
    autonHandler.runAuto();
  }

	@Override
	public void teleopInit() {
	  
	  //ends auto if it has been started
	  if(!init) {
	    autonHandler.endAuto();
	    init = true;
	  }
	  
	  //
    compres.start();

	  //start loops
    enabledLooper.startLoops();

    //stop everything
	  intake.stop();
	  driveTrain.setStop();
	  elevator.moveSmooth(elevator.getHeight(), 10, 2);
	}


	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
	  driveTrain.setTeleop(true);
	  //cameraBoyo.hAngleToTarget();

	 if(!climber.isClimbing()) {
	  driveTrain.teleopDrive(leftJ.getYBanded(), -rightJ.getXBanded());
	 } else {
	   driveTrain.setStop();
	 }
	  //elevator control
	  if(rightJ.getRisingEdge(5)) {
	    
	    elevator.moveSmooth(30, 80 , 20);
	  }  else if(rightJ.getRisingEdge(6)) {
	    
	    elevator.moveSmooth(59, 80 , 20);
    }  else if(rightJ.getRisingEdge(4)) {
      
      elevator.moveSmooth(67, 80 , 20);
    } else if(rightJ.getRisingEdge(3)) {
      elevator.moveSmooth(1 , 80, 20);
	  }
	  
	  if(!elevator.moving() && !(rightJ.getPOV() == 0) && !(rightJ.getPOV() == 180)) {
	    elevator.stopElevator();
	  }
	  
	  if(rightJ.getPOV() == 0) {
	    
	    elevator.moveUnsafe(0.65);
	    //elevator.moveSmooth(elevator.getHeight()+12, 30, 10);
	  } else if(rightJ.getPOV() == 180) {
	    elevator.moveUnsafe(-0.3);
	    //elevator.moveSmooth(elevator.getHeight()-12, 30, 10);
	  }

	 
	 if(rightJ.getRawButton(1)) {
	   intake.succ();
	 } else if(rightJ.getRawButton(2)) {
	   intake.spit();
	 } else if(leftJ.getRawButton(2)) {
	   intake.outakeAuto();
	 } else {
	   
	   intake.stop();
	 }

	 if(operatorJ.getRisingEdge(1)) {
	   climber.readyToClimb();
	 } else if(operatorJ.getRisingEdge(3)) {
	   Robot.getDriveTrain().engagePTO();
	   climber.climb();
	 } 

	 if(operatorJ.getRawButton(4)) {
	   climber.releaseBackpack();
	 }

	 if(operatorJ.getRisingEdge(2)) {
	   climber.force(0.5);
	 }

	 if(operatorJ.getFallingEdge(6) || operatorJ.getFallingEdge(2) || operatorJ.getFallingEdge(3)) {
     climber.stopClimb();
	 }
	 
	 if(operatorJ.getRisingEdge(6)) {
     climber.force(-0.55);
	 }

	 if(operatorJ.getRawButton(5)) {
	   climber.stopClimb();
	 }

	}


	public static Drivetrain getDriveTrain() {
	  return driveTrain;
	}


	public static Intake getIntake() {
	  return intake;
	}

	public static Elevator getElevator() {
    return elevator;
  }

	public static Climber getClimber() {
	  return climber;
	}

	public static CameraBoi getCameraBoyo() {
	  return cameraBoyo;
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
}

