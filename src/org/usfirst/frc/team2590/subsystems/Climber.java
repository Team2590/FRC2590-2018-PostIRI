package org.usfirst.frc.team2590.subsystems;

import org.usfirst.frc.team2590.controls.ControlPresets;
import org.usfirst.frc.team2590.controls.EnhancedProfileCreator;
import org.usfirst.frc.team2590.robot.Robot;
import org.usfirst.frc.team2590.robot.RobotMap;
import org.usfirst.frc.team2590.util.AveragingQueue;
import org.usfirst.frc.team2590.util.DelayedBoolean;
import org.usfirst.team2590.settings.ClimberSettings;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Climber backpack combo
 * @author Connor_Hofenbitzer
 *
 */
public class Climber implements RobotMap, ClimberSettings {

  private static Climber climberInstance = null;
  public static Climber getClimberInstance() {
    if(climberInstance == null) {
      climberInstance = new Climber();
    }
    return climberInstance;
  }
  private States climberState = States.NULL;
  private enum States {
    NULL, UNWINDING , RETRACTING , FORCE
  }

  private double forceSpeed;
  private final double sprocketDiameter = 1.4; //inches

  //climber motor
  private VictorSPX climberMotor;

  //pistons
  private Solenoid ratchet;
  private Solenoid backpack;

  //sensors
  private Encoder climberEncoder;

  //possible sets of control gains
  private ControlPresets extend;
  private ControlPresets halfLoad;
  private ControlPresets heavyLoad;
  private ControlPresets lightLoad;

  //actual motion profiler
  private EnhancedProfileCreator profiler;

  //system check stuff
  private AveragingQueue systemQueue;
  private DelayedBoolean ratchetFuse;

  //checks if the climb has been initiated
  private boolean hasStarted;

  public Climber() {

    forceSpeed = 0;
    hasStarted = false;

    backpack = new Solenoid(BackPackSol);
    ratchet = new Solenoid(ClimberRatchetSol);

    //init the motor
    climberMotor = new VictorSPX(ClimberMID);
    climberMotor.setInverted(true);

    climberEncoder = new Encoder(climberEncoderA, climberEncoderB);

    climberEncoder.setReverseDirection(true);
    climberEncoder.setName("Climber encoder");
    climberEncoder.setDistancePerPulse(1./360. * (sprocketDiameter * Math.PI));

    //control gains
    extend = new ControlPresets(UPKP, UPKF, UPKA, MAXPREVEL, MAXPREACCEL);
    halfLoad = new ControlPresets(MIDKP, MIDKF, MIDKA, MAXVEL, MAXACCEL);
    lightLoad = new ControlPresets(LOWKP, LOWKF, LOWKA, MAXVEL, MAXACCEL);
    heavyLoad = new ControlPresets(HIGHKP, HIGHKF, HIGHKA, MAXVEL, MAXACCEL);

    //motion profiler
    profiler = new EnhancedProfileCreator(extend);

    //pre match stuff
    systemQueue = new AveragingQueue(AVERAGING_QUEUE_LENGTH);
  }

  public void update(double dt) {
    switch(climberState) {
      case NULL :
        ratchet.set(false);
        SmartDashboard.putBoolean("DB/LED 1", true);
        if(!hasStarted) {
          Robot.getDriveTrain().disEngagePTO();
        } else {
          Robot.getDriveTrain().engagePTO();
          Robot.getDriveTrain().getLeftDrive().set(ControlMode.PercentOutput, 0);
          Robot.getDriveTrain().getRightDrive().set(ControlMode.PercentOutput, 0);
        }
        climberMotor.set(ControlMode.PercentOutput, 0);
        break;

        //extends the elevator
      case UNWINDING :

        if(!ratchetFuse.getDone()) {
          ratchet.set(true);
        } else {
          SmartDashboard.putBoolean("DB/LED 1", false);

          scheduleGains(0, true);
          Robot.getDriveTrain().disEngagePTO();

          if(!profiler.isDone()) {
            double pwr = profiler.calculate(getDistance());
            climberMotor.set(ControlMode.PercentOutput, pwr);
          } else {
            setStop();
          }
        }
        break;

      case FORCE:
        ratchet.set(true);
        if(ratchetFuse.getDone()) {
          if(hasStarted) {
           Robot.getDriveTrain().getLeftDrive().set(ControlMode.PercentOutput, forceSpeed);
           Robot.getDriveTrain().getRightDrive().set(ControlMode.PercentOutput, forceSpeed);
          }
          climberMotor.set(ControlMode.PercentOutput, forceSpeed);
        }

        break;

     
    }
  }

  public boolean isClimbing() {
    return hasStarted;
  }
  /**
   * Open the backpack
   */
  public void releaseBackpack() {
    backpack.set(true);
  }
  
 

  public void force(double speed) {
    ratchetFuse = new DelayedBoolean(0.1);
    forceSpeed = speed;
    climberState = States.FORCE;
  }
  /**
   * Sets the elevator setpoint
   * @param setpoint : setpoint for the climber elevator
   */
  public void climbExtendTo(double setpoint) {
    profiler.setSetpoint(getDistance(), setpoint);
  }

  /**
   * Changes the gains based off of the climb type
   * @param amountOfRobots : 0 for just yourself, 1 for one robot next to you, 2 for carrying alliance
   * @param isExtending : if its extending this should be true
   */
  public void scheduleGains(int amountOfRobots, boolean isExtending) {
    if(isExtending) {
      profiler.changeGains(extend);
    } else {

      switch(amountOfRobots) {
        case 0:
          profiler.changeGains(lightLoad);
          break;
        case 1:
          profiler.changeGains(halfLoad);
          break;
        case 2:
          profiler.changeGains(heavyLoad);
          break;
        default:
          profiler.changeGains(heavyLoad);
          DriverStation.reportWarning(amountOfRobots + " is greater than 2 or less than 0, this not allowed", false);
      }

    }
  }


  /**
   * Stops climbing
   */
  public void setStop() {
    climberState = States.NULL;
  }

  /**
   * Extends the climber and gets ready to climb
   */
  public void readyToClimb() {
    climbExtendTo(CLIMB_HEIGHT);
    ratchetFuse = new DelayedBoolean(0.1);
    climberState = States.UNWINDING;
  }

  /**
   * Starts climbing
   */
  public void climb() {
    hasStarted = true;
    ratchetFuse = new DelayedBoolean(0.1);
    force(-1);
  }

  public void stopClimb() {
    climberState = States.NULL;
  }
  /**
   * Gets the main motor
   * @return : master motor
   */
  public VictorSPX getMasterMotor() {
    return climberMotor;
  }

  public double getDistance() {
    return climberEncoder.getDistance();
  }

  /**
   * Nice system checks
   */
  public void runSystemChecks() {
    try {
      readyToClimb();
      systemQueue.update(climberMotor.getOutputCurrent());
      SmartDashboard.putNumber("Climber current average", systemQueue.getAverage());
    } catch(Exception e) {
      DriverStation.reportError("cant communicate the climber current at this time", false);
    }
  }
}
