package org.usfirst.frc.team2590.subsystems;

import org.usfirst.frc.team2590.controls.BangBangController;
import org.usfirst.frc.team2590.controls.ConstantCurrent;
import org.usfirst.frc.team2590.controls.ControlPresets;
import org.usfirst.frc.team2590.controls.Controller;
import org.usfirst.frc.team2590.controls.EnhancedProfileCreator;
import org.usfirst.frc.team2590.controls.VelocityController;
import org.usfirst.frc.team2590.robot.RobotMap;
import org.usfirst.frc.team2590.util.AveragingQueue;
import org.usfirst.frc.team2590.util.NemesisEncoder;
import org.usfirst.team2590.settings.ElevatorSettings;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * Elevator subsystem, multiple controllers
 * @author Connor_Hofenbitzer
 *
 */
public class Elevator implements ElevatorSettings , RobotMap {

  //singleton
  private static Elevator elevatorInstance = null;
  public static Elevator getElevatorInstance() {
    if(elevatorInstance == null) {
      elevatorInstance = new Elevator();
    }
    return elevatorInstance;
  }

  //possible states the elevator may be in
  private States elevatorState = States.STOPPED;
  private enum States {
    STOPPED , MOVING
  }

  //used to calculate distance per pulse
  private final double sprocketDiameter = 1.273;

  //controller
  private Controller ElevatorController;

  //motors
  private VictorSPX elevatorMotor;

  //encoder
  private NemesisEncoder elevatorEncoder;

  //limit switches
  //private DigitalInput topSwitch;
  private DigitalInput bottomSwitch;

  //control stuff
  private ControlPresets motionProfilePres;

  //system check stuff
  private double setpoint;
  private AveragingQueue currentAverage;

  public Elevator() {

    //init the elevator controller
    setpoint = 0;
    ElevatorController = null;
    motionProfilePres  = new ControlPresets(ELEVATOR_KP, ELEVATOR_KF, ELEVATOR_KA, 0, 0);

    //motors
    elevatorMotor = new VictorSPX(ElevatorID);
    elevatorMotor.setInverted(false);
    elevatorMotor.setNeutralMode(NeutralMode.Brake);

    //limit
    //topSwitch = new DigitalInput(TopLimitPIN);
    bottomSwitch = new DigitalInput(BottomLimitPIN);
    bottomSwitch.setName("Bottom Limit");

    //encoder
    elevatorEncoder = new NemesisEncoder(ElevatorEncoderA, ElevatorEncoderB);

    elevatorEncoder.setDistancePerPulse( (1./360. * (sprocketDiameter * Math.PI) )*2);
    elevatorEncoder.setName("Elevator Encoder");

    //current averager
    currentAverage = new AveragingQueue(AVERAGING_QUEUE_LENGTH);
  }

  /**
   * Updates the System
   * @param dt : Gives difference in time between one call to update and another
   */
  public void update(double dt) {

    switch(elevatorState) {
      case STOPPED:

        elevatorMotor.setNeutralMode(NeutralMode.Brake);
        SmartDashboard.putBoolean("DB/LED 2", true);

        double powr = 0.0;
        if(!bottomSwitch.get()) {
          elevatorEncoder.reset();
          setpoint = 0;
        }
        
        if(getHeight() > 2 && setpoint != 0) {
          double error = setpoint - getHeight();
          powr = error * ELEVATOR_CONSTANT;
        }
        elevatorMotor.set(ControlMode.PercentOutput, powr );
        break;

      case MOVING:
        elevatorMotor.setNeutralMode(NeutralMode.Brake);
        SmartDashboard.putBoolean("DB/LED 2", false);
        if(ElevatorController instanceof ConstantCurrent) {
          setpoint = getHeight();
        }
        
        //if were done moving then dont move
        if(ElevatorController.isDone() || (!bottomSwitch.get() && setpoint < 10 ) ) {
          elevatorState = States.STOPPED;
        }

        double output = ElevatorController.calculate(getHeight());
        elevatorMotor.set(ControlMode.PercentOutput, output);

        break;
    default:
      DriverStation.reportWarning("HIT DEFAULT ELEVATOR CASE", false);
    }


  }

  /**
   * Sets the systems setpoint
   * @param setpoint : where you want the system to go
   */
  private void setSetpoint(double setpoint) {

    //just restricts the output
    if(setpoint < MIN_HEIGHT || setpoint > MAX_HEIGHT) {
      setpoint = (Math.abs(setpoint - MIN_HEIGHT) < Math.abs(setpoint - MAX_HEIGHT) )
          ? MIN_HEIGHT : MAX_HEIGHT;
    }

    this.setpoint = setpoint;
    if(ElevatorController instanceof EnhancedProfileCreator) {
      ElevatorController.setSetpoint(getHeight(), setpoint);

    } else {
      ElevatorController.setSetpoint(setpoint);
    }
    elevatorState = States.MOVING;
  }

  /**
   * Moves with a constant voltage, not recomended
   * @param volts: voltage to run the motor at
   */
  public void moveUnsafe(double volts) {
    ElevatorController = new ConstantCurrent();
    ElevatorController.setSetpoint(volts);
    
    elevatorState = States.MOVING;
  }
  /**
   * Moves using motion profiling, slower but smoother and more accurate
   * @param stp : where you want the system to go
   */
  public void moveSmooth(double stp, double vel, double acc) {
    motionProfilePres = new ControlPresets(ELEVATOR_KP, ELEVATOR_KF, ELEVATOR_KA, vel, acc);
    ElevatorController = new EnhancedProfileCreator(motionProfilePres);
    setSetpoint(stp);
  }
  
  public boolean moving() {
    return elevatorState == States.MOVING && (ElevatorController instanceof EnhancedProfileCreator); 
  }
  /**
   * Gets there quickly , not quite accuratly
   * @param stp : where you want the system to go
   */
  public void moveQuick(double stp) {
    ElevatorController = new BangBangController(ELEVATOR_CONTROL_VOLT, ELEVATOR_CONTROL_TOLERANCE);
    setSetpoint(stp);
  }


  /**
   * Stops the elevator
   */
  public void stopElevator() {
    elevatorState = States.STOPPED;
  }

  /**
   * Gets the currents elevator height
   * @return : the distance from the bottom of the carrige
   */
  public double getHeight() {
      return elevatorEncoder.getDistance();
  }

  public double getSetpoint() {
    return setpoint;
  }


  /**
   * Checks if either limit has been hit
   * @return : if a limit has been hit
   */
  public boolean limitHit() {
    return !bottomSwitch.get();
  }

  /**
   * Sets the current location of the encoderp
   * @param set : location the encoder should think it is at
   */
  public void setEncoder(double set) {
    elevatorEncoder.setDistance(set);
  }

  /**
   * checks if the carrige has reached its setpoint
   * @return : if the carrige is near its setpoint
   */
  public boolean isDone() {
    return ElevatorController.isDone();
  }

  /**
   * Runs the elevator system checks
   */
  public void runSystemChecks() {
    try {

      currentAverage.update(elevatorMotor.getOutputCurrent());
      SmartDashboard.putNumber("Elevator current", currentAverage.getAverage());
    } catch(Exception e) {
      DriverStation.reportError("cant communicate the elevator current at this time", false);
    }
  }


}

