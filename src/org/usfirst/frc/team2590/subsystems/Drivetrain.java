package org.usfirst.frc.team2590.subsystems;

import org.usfirst.frc.team2590.controls.ControlPresets;
import org.usfirst.frc.team2590.controls.Controller;
import org.usfirst.frc.team2590.controls.EnhancedProfileCreator;
import org.usfirst.frc.team2590.controls.NemesisMultiMC;
import org.usfirst.frc.team2590.controls.NemesisTalonSRX;
import org.usfirst.frc.team2590.controls.PIDTurnController;
import org.usfirst.frc.team2590.controls.VelocityController;
import org.usfirst.frc.team2590.robot.Robot;
import org.usfirst.frc.team2590.robot.RobotMap;
import org.usfirst.frc.team2590.util.AveragingQueue;
import org.usfirst.frc.team2590.util.NemesisDrive;
import org.usfirst.team2590.settings.DriveTrainSettings;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * <h1> Subsystem </h1>
 *
 *    The DriveTrain class for Festus
 *
 * <h2> Features: </h2>
 * <ul>
 *    <li>Motion Profiled Turning</li>
 *    <li>Motion Profiled Drive Base</li>
 * </ul>
 *
 *
 * @version 1.0
 * @author Connor_Hofenbitzer
 *
 */
public class Drivetrain implements DriveTrainSettings , RobotMap {

	//new singleton
	private static Drivetrain driveInstance = null;
	public static Drivetrain getDriveInstance() {
		if(driveInstance == null) {
			driveInstance = new Drivetrain();
		}
		return driveInstance;
	}

	private States driveState = States.STOPPED;
	private enum States {
		STOPPED , TELEOP_DRIVE , PATH_FOLLOWING , 
		TURN , DRIVE_STRAIGHT, VELOCITY
	}

	//motors
	private int cycles = 0;
	private boolean inTeleop;
	private double driveSetpoint;

	private NemesisTalonSRX leftDriveMaster;
	private NemesisTalonSRX rightDriveMaster;

	private VictorSPX leftDriveSlave;
	private VictorSPX rightDriveSlave;

	//sensors
	private ADXRS450_Gyro gyro;
	private Encoder leftDriveEncoder;
	private Encoder rightDriveEncoder;

	private boolean latchingBoolean;
	
	//driveTrain
  private double turnPower;
  private NemesisDrive driver;
  private double straightPower;
  private Controller currentController;

  //power take off piston, converts drive power to climber power
  private Solenoid PTO;
  private boolean turnDne;
  
  //master checks
  private AveragingQueue currentAverageLeftM;
  private AveragingQueue currentAverageRightM;

  //slave checks
  private AveragingQueue currentAverageLeftS;
  private AveragingQueue currentAverageRightS;

  private NemesisMultiMC motorController;
  private PIDTurnController turn;
  
  private EnhancedProfileCreator leftSideController;
  private EnhancedProfileCreator rightSideController;
  
  private double fixedHeading = 0;
  private boolean forward;
  private boolean lastOn;
  
	public Drivetrain() {

	  //variables
	  turnPower = 0;
	  lastOn = false;
	  turnDne = false;
	  inTeleop = false;
	  forward = false;
	  straightPower = 0;
	  driveSetpoint = 0;
	  currentController = new VelocityController(0.1, 0.053);

	  //motors
		leftDriveMaster = new NemesisTalonSRX(leftSideDriveMID);
		leftDriveSlave = new VictorSPX(leftSideDriveSID);

		rightDriveMaster = new NemesisTalonSRX(rightSideDriveMID);
		rightDriveSlave = new VictorSPX(rightSideDriveSID);

		leftDriveSlave.follow(leftDriveMaster);
		rightDriveSlave.follow(rightDriveMaster);

		setAllMode(true);

		driver = new NemesisDrive();

		//sensors
		gyro = new ADXRS450_Gyro(); //$"Gyro on the Fetus hahahaha" - Mr. Wolfe

		leftDriveEncoder = new Encoder(leftSideEncoderB, leftSideEncoderA);
		rightDriveEncoder = new Encoder(rightSideEncoderB, rightSideEncoderA);

		leftDriveEncoder.setDistancePerPulse(1.0/360.0 * (WheelDiameter * Math.PI));
    rightDriveEncoder.setDistancePerPulse(1.0/360.0 * (WheelDiameter * Math.PI));

    leftDriveEncoder.setName("left drive encoder");
    leftDriveEncoder.setReverseDirection(true);
    rightDriveEncoder.setName("right drive encoder");

    leftDriveMaster.setInverted(true);
    leftDriveSlave.setInverted(true);

    rightDriveMaster.setInverted(false);
    rightDriveSlave.setInverted(false);

    latchingBoolean = false;
    //piston to engage and disengage the pto (power take off)
    PTO = new Solenoid(PTOSol);

    leftSideController = new EnhancedProfileCreator(new ControlPresets(straightProfileKP, straightProfileKF, 0.0001,
                                                                      maxVelStraight, maxAccStraight));
    rightSideController = new EnhancedProfileCreator(new ControlPresets(straightProfileKP, straightProfileKF, 0.0001,
        maxVelStraight, maxAccStraight));

    //system checks
    currentAverageLeftM = new AveragingQueue(AVERAGING_QUEUE_LENGTH);
    currentAverageLeftS = new AveragingQueue(AVERAGING_QUEUE_LENGTH);

    currentAverageRightM = new AveragingQueue(AVERAGING_QUEUE_LENGTH);
    currentAverageRightS = new AveragingQueue(AVERAGING_QUEUE_LENGTH);

    
    leftDriveMaster.configPeakCurrentLimit(0, 10);
    rightDriveMaster.configPeakCurrentLimit(0, 10);
    
    leftDriveMaster.configPeakCurrentDuration(200, 10);
    rightDriveMaster.configPeakCurrentDuration(200, 10);

    
    leftDriveMaster.configContinuousCurrentLimit(35, 10);
    rightDriveMaster.configContinuousCurrentLimit(35, 10);
  
    setAllMode(false);
    leftDriveMaster.enableCurrentLimit(false);
    rightDriveMaster.enableCurrentLimit(false);
    
    
    motorController = new NemesisMultiMC(leftDriveMaster, rightDriveMaster);
    turn = new PIDTurnController(0.007, 0.0, 0.0075, gyro, motorController);

    turn.setBreakFreePoint(0.25);

    turn.setLockupPoint(0.225);
    
    turn.setTurningDistinctionPoint(2);
	}


  /**
   * Updates the System
   * @param timeDelta : Gives difference in time between one call to update and another
   */
	public void loop(double timeDelta) {
	
    setAllMode(true);
    //System.out.println("angle " + gyro.getAngle() + " " + getLeftEncoder().getDistance() + " " + getRightEncoder().getDistance());
    switch(driveState) {
		  case STOPPED :
		    cycles = 0;
		    turnDne = false;
		    //System.out.println("Latch Bool: " + latchingBoolean);
		    //if(latchingBoolean) {
		      turn.disable();
		      latchingBoolean = false;
		    //}

		    break;

		  case TELEOP_DRIVE :
		    if(Robot.getClimber().isClimbing()) {
		      toggleCurrentLimiting(false);
		    } else {
		      toggleCurrentLimiting(true);
		    }
		    
		    double[] out = driver.calculate(straightPower, turnPower);
        setSpeeds(out[0], out[1]);
		   
		    break;

		  case DRIVE_STRAIGHT :
       toggleCurrentLimiting(true);
		   if(leftSideController.isDone() && rightSideController.isDone()) {
		     driveState = States.STOPPED;
		   }

		   //keeps a forward heading
	     double angularError =  (fixedHeading-gyro.getAngle()) * straightProfileKH;

		   setSpeeds( - (leftSideController.calculate(getLeftEncoder().getDistance()) + angularError),
		       -(rightSideController.calculate(getRightEncoder().getDistance()) - angularError) );


		    break;

		  case PATH_FOLLOWING :
	       toggleCurrentLimiting(true);
		    //basically just dont annoy the motors
		    break;
		  case TURN :
        toggleCurrentLimiting(true);
		    
		    //System.out.println("gyro " + gyro.getAngle());
        double error = (driveSetpoint - gyro.getAngle());
        turnDne = Math.abs(error) <  2.5;
        if(turnDne) {
          cycles += 1;
          if(cycles > 10) {
            System.out.println("done");
            driveState = States.STOPPED;
            break;
          }
        } else {
          cycles = 0;
        }
        
		    break;
		  case VELOCITY :
		      //drive at a constant velocity
          toggleCurrentLimiting(false);
		      setSpeeds(-driveSetpoint*velConstant, -driveSetpoint*velConstant);
		    break;
		   
        
		  default:

		    break;
		}
    
    
	}
	
	public void reversePath(boolean rev) {
    leftDriveEncoder.setReverseDirection(!rev);
    rightDriveEncoder.setReverseDirection(rev);
	}

	public void toggleCurrentLimiting(boolean on) {
	  if(lastOn != on) {
	    leftDriveMaster.enableCurrentLimit(on);
	    rightDriveMaster.enableCurrentLimit(on);
	    lastOn = on;
	  }
	 // System.out.println("current Toggle limit " + on);
	}
	

	public void setTeleop(boolean teleop) {
	  inTeleop = teleop;
	  leftDriveMaster.setInverted(true);
    leftDriveSlave.setInverted(true);

    rightDriveMaster.setInverted(false);
    rightDriveSlave.setInverted(false);
	  if(teleop) {
	    resetAllSensors();
	    driveSetpoint = 0;
	  }

	}

	/**
	 * Stops the drivetrain all together
	 */
	public void setStop() {
	  driveState = States.STOPPED;
	}

	/**
	 * Turns to a specified setpoint
	 * @param setpoint : angle to turn to - degrees
	 * @param reset : reset the sensors
	 */
	public void turn(double setpoint, boolean reset, boolean quick) {
	  if(reset) {
      resetAllSensors();
    }
	  latchingBoolean = true;
	  rightDriveMaster.configVoltageCompSaturation(9.5, 10);
    leftDriveMaster.configVoltageCompSaturation(9.5, 10);
    
    rightDriveMaster.enableVoltageCompensation(true);
    leftDriveMaster.enableVoltageCompensation(true);
    
    leftDriveMaster.setInverted(false);
    leftDriveSlave.setInverted(false);

    rightDriveMaster.setInverted(false);
    rightDriveSlave.setInverted(false);
    //turn = new TurningController(new ControlPresets(SmartDashboard.getNumber("DB/Slider 0", 0.0), SmartDashboard.getNumber("DB/Slider 1", 0.0), SmartDashboard.getNumber("DB/Slider 2", 0.0), 40, 25)); //0.025 0.021 0.0001
    //turnCont.setSetpoint(setpoint, gyro.getAngle());
    driveSetpoint = setpoint;

    turn.setSetpoint(setpoint);
    turn.enable();

    driveState = States.TURN;
	}

	 
	public boolean turnDone() {
	  //return turnDne && cycles >= 9;
	  return  driveState == States.STOPPED ;
	}

	/**
	 * checks if the drive straight is done
	 * @return : completed drive straight
	 */
	public boolean driveStraightDone() {
	  return ( leftSideController.isDone() && rightSideController.isDone() ) || inTeleop;
	}

	/**
	 * Runs the drivetrain at a certain velocity
	 * @param velocity : velocity to drive at
	 */
	public void setVelocity(double velocity) {
	  leftDriveMaster.setInverted(true);
    leftDriveSlave.setInverted(true);

    rightDriveMaster.setInverted(false);
    rightDriveSlave.setInverted(false);
	  driveSetpoint = velocity;
	  currentController.setSetpoint(velocity);
	  driveState = States.VELOCITY;
	}

	/**
	 * Sets all the motors to a specified mode
	 * @param breakM : break or coast
	 */
	public void setAllMode(boolean breakM) {
	  leftDriveMaster.setNeutralMode(breakM ? NeutralMode.Brake : NeutralMode.Coast);
	  rightDriveMaster.setNeutralMode(breakM ? NeutralMode.Brake : NeutralMode.Coast);

	  leftDriveSlave.setNeutralMode(breakM ? NeutralMode.Brake : NeutralMode.Coast);
	  rightDriveSlave.setNeutralMode(breakM ? NeutralMode.Brake : NeutralMode.Coast);

	}

	/**
	 * Drives at a given acceleration and velocity to the positional setpoint
	 * @param setpoint : distance to drive to
	 */
	public void driveStraight(double setpoint, boolean quick, double angle) {
	  resetDrive();
	  rightDriveMaster.enableVoltageCompensation(false);
    leftDriveMaster.enableVoltageCompensation(false);
    leftDriveMaster.setInverted(true);
    leftDriveSlave.setInverted(true);

    rightDriveMaster.setInverted(false);
    rightDriveSlave.setInverted(false);
    
    //reschedules control gains based on if you want to go quickly
	  if(quick) {
	    leftSideController = new EnhancedProfileCreator(new ControlPresets(straightProfileKP, straightProfileKF, 0.0001,
          superMaxVelStraight, superMaxAccStraight));
      rightSideController = new EnhancedProfileCreator(new ControlPresets(straightProfileKP, straightProfileKF, 0.0001,
          superMaxVelStraight, superMaxAccStraight));
	  } else {
	    leftSideController = new EnhancedProfileCreator(new ControlPresets(straightProfileKP, straightProfileKF, 0.0001,
          maxVelStraight, maxAccStraight));
	    rightSideController = new EnhancedProfileCreator(new ControlPresets(straightProfileKP, straightProfileKF, 0.0001,
	        maxVelStraight, maxAccStraight));
	  }
	  
	  fixedHeading = gyro.getAngle();
	  driveSetpoint = setpoint;
	  leftSideController.setSetpoint(getLeftEncoder().getDistance(), setpoint);
	  rightSideController.setSetpoint(getRightEncoder().getDistance(), setpoint);
	  driveState = States.DRIVE_STRAIGHT;
	}
	/**
	 * Gets the left drive encoder
	 * @return : robots left drive encoder
	 */
	public Encoder getLeftEncoder() {
	  return leftDriveEncoder;
	}

	/**
	 * Gets the right drive Encoder
	 * @return : the robots right drive encoder
	 */
	public Encoder getRightEncoder() {
	  return rightDriveEncoder;
	}

	/**
	 * Gets the robots gyro
	 * @return : the gyro spartan board
	 */
	public PIDSource getGyro() {
	  return gyro;
	}

	public void resetDrive() {
	  leftDriveEncoder.reset();
	  rightDriveEncoder.reset();
	}
	/**
	 * Resets all the drive sensors
	 */
	public void resetAllSensors() {
	  gyro.reset();
	  leftDriveEncoder.reset();
	  rightDriveEncoder.reset();
	}

	/**
	 * Gets the drive motors for the left side of the drive train
	 * @return : left drive motors chained together
	 */
	public TalonSRX getLeftDrive() {
	  return leftDriveMaster;
	}

	/**
	 * Gets the drive motors for the right side of the drive train
	 * @return : right drive motors chained together
	 */
	public TalonSRX getRightDrive() {
	  return rightDriveMaster;
	}

	/**
	 * Runs a chosen path
	 */
	public void runPath() {
	  rightDriveMaster.enableVoltageCompensation(false);
    leftDriveMaster.enableVoltageCompensation(false);
	  leftDriveMaster.setInverted(false);
    leftDriveSlave.setInverted(false);

    rightDriveMaster.setInverted(true);
    rightDriveSlave.setInverted(true);
	  driveState = States.PATH_FOLLOWING;
	}

	/**
	 *
	 * set the individual speeds of the motors
	 * @param left : left side percent -1 to 1
	 * @param right : right side percent -1 to 1
	 */
	private void setSpeeds(double left, double right) {
	  leftDriveMaster.set(ControlMode.PercentOutput  , left);
	  rightDriveMaster.set(ControlMode.PercentOutput  , right);
	}

	/**
	 * Drives in teleop
	 * @param straight : forward and backwards power -1 to 1
	 * @param turn 	   : left and right power -1 to 1
	 */

	public void teleopDrive(double straight , double turn) {
	  turnPower = turn;
	  straightPower = straight;

	  leftDriveMaster.setInverted(true);
    leftDriveSlave.setInverted(true);

    rightDriveMaster.setInverted(false);
    rightDriveSlave.setInverted(false);
	  driveState = States.TELEOP_DRIVE;
	}

	/**
	 * Get every drive motor to follow a single motor
	 * @param motor : the motor all these other motors should follow
	 */
	public void setAllToFollow(VictorSPX motor) {
    leftDriveSlave.follow(motor);
    leftDriveMaster.follow(motor);

	  rightDriveSlave.follow(motor);
	  rightDriveMaster.follow(motor);
	}


	/**
	 * Engages the PTO for the climber
	 */
	public void engagePTO() {
	  //makes sure we dont keep on trying to engage the piston
	  if(!PTO.get()) {
	    PTO.set(true);
	  }
	}

	/**
	 * Disengages the PTO for the climber
	 */
	public void disEngagePTO() {
	   //makes sure we dont keep on trying to disengage the piston
	  if(PTO.get()) {
	    PTO.set(false);
	  }
  }

	/**
	 * This is mainly here to make sure we don't have a 2017 worlds repeat
	 * for those of us who don't know what happened at the infamous 2017 worlds Archimedes field...
	 * a single drive motor was acting up, and it was dismissed as a network issue, we ended up
	 * loosing world champs due to this, an issue which could have been deduced by checking current draw
	 */
	public void runSystemChecks() {
	  try {

	    //run all the motors forward
	    teleopDrive(1 , 0);

	    //checks the left side
	    currentAverageLeftS.update(leftDriveSlave.getOutputCurrent());
	    currentAverageLeftM.update(leftDriveMaster.getOutputCurrent());
	    double leftSideDifference = Math.abs(currentAverageLeftS.getAverage() - currentAverageLeftM.getAverage());

	    //send all the left side stuff
	    SmartDashboard.putNumber("Left Side Current Difference", leftSideDifference);
	    SmartDashboard.putNumber("Left Side Current Master", currentAverageLeftM.getAverage());
	    SmartDashboard.putNumber("Left Side Current Slave",  currentAverageLeftS.getAverage());

	    //checks the right side
	    currentAverageRightS.update(rightDriveSlave.getOutputCurrent());
	    currentAverageRightM.update(rightDriveMaster.getOutputCurrent());
	    double rightSideDifference = Math.abs(currentAverageRightS.getAverage() - currentAverageRightM.getAverage());

	    //send all the right side stuff
	    SmartDashboard.putNumber("Right Side Current Difference", rightSideDifference);
      SmartDashboard.putNumber("Right Side Current Master", currentAverageRightM.getAverage());
      SmartDashboard.putNumber("Right Side Current Slave",  currentAverageRightS.getAverage());

	    // preventing a 2017 worlds for the left side
	    if(leftSideDifference > leftSideAverageTolerance &&
	            currentAverageLeftM.getCurrentSize() > AVERAGING_QUEUE_LENGTH/2) {

	      DriverStation.reportError("HEY THE LEFT SIDE IS BEING WIERD MASTER : " + currentAverageLeftM.getAverage() +
	          " SLAVE : " + currentAverageLeftS.getAverage() + " DIFFERENCE " + leftSideDifference, false);
	    }


      // preventing a 2017 worlds for the right side
      if(rightSideDifference > rightSideAverageTolerance &&
              currentAverageRightM.getCurrentSize() > AVERAGING_QUEUE_LENGTH/2) {

        DriverStation.reportError("HEY THE RIGHT SIDE IS BEING WIERD MASTER : " + currentAverageRightM.getAverage() +
            " SLAVE : " + currentAverageRightS.getAverage() + " DIFFERENCE " + rightSideDifference , false);
      }



	  } catch (Exception e) {
	    DriverStation.reportError("cant communicate the drivetrain current at this time" , false);
	  }
	}

}

