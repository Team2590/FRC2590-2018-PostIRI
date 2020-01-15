package org.usfirst.frc.team2590.subsystems;

import org.usfirst.frc.team2590.robot.RobotMap;
import org.usfirst.frc.team2590.util.AveragingQueue;
import org.usfirst.team2590.settings.IntakeSettings;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Intake subsystem, aquires boxes
 * @author Chinmeme Savanur
 * @author Connor_Hofenbitzer (edited)
 * @version 1.2
 */
public class Intake implements RobotMap, IntakeSettings {

  //singleton
  private static Intake intakeInstance = null;
  public static Intake getIntakeInstance() {
    if(intakeInstance == null) {
      intakeInstance = new Intake();
    }
    return intakeInstance;
  }

  //possible states the intake could be in
  private States intakeState = States.STOPPED;
  private enum States {
    STOPPED, INTAKE_IN, INTAKE_OUT , AUTON_INTAKE, AUTON_OUT, SUPER
  }

  //motors
  private VictorSPX intakeMotor_left;
  private VictorSPX intakeMotor_right;

  //sensors
  private AnalogInput leftBoxDetect;
  private AnalogInput rightBoxDetect;

  private AveragingQueue systemAverageL;
  private AveragingQueue systemAverageR;

  public Intake() {
    // Creating classes for hardware is done in init() method.
    // This allows for using mock classes in testing, and real classes with the robot.
  }

  /* Initialize, specifying instances of hardware classes.   Useful for mocks / testing */
  public void init(VictorSPX il, VictorSPX ir, AnalogInput lb, AnalogInput rb) {
    intakeMotor_left = il;
    intakeMotor_right = ir;
    leftBoxDetect = lb;
    rightBoxDetect = rb;

    this.configure();
  }

   /* Default initializer, will init robot hardware */
   public void init() {

        //motors
        intakeMotor_left = new VictorSPX(leftIntakeID);
        intakeMotor_right = new VictorSPX(rightIntakeID);
  
        //sensors
        leftBoxDetect = new AnalogInput(leftBoxDetectorPin);
        rightBoxDetect = new AnalogInput(rightBoxDetectorPin);

        this.configure();
    
  }
  public void configure() {
    intakeMotor_left.setNeutralMode(NeutralMode.Coast);
    intakeMotor_right.setNeutralMode(NeutralMode.Coast);

    intakeMotor_left.setInverted(false);
    intakeMotor_right.setInverted(false);

    //sets gains for current control
    intakeMotor_left.config_kF(0, leftKF, 10);
    intakeMotor_left.config_kP(0, leftKP, 10);
    intakeMotor_left.config_kI(0, leftKI, 10);
    intakeMotor_left.config_kD(0, leftKD, 10);

    intakeMotor_right.config_kF(1, rightKF, 10);
    intakeMotor_right.config_kP(1, rightKP, 10);
    intakeMotor_right.config_kI(1, rightKI, 10);
    intakeMotor_right.config_kD(1, rightKD, 10);

    intakeMotor_left.selectProfileSlot(0, 0);
    intakeMotor_right.selectProfileSlot(0, 1);
        
    leftBoxDetect.setName( "Left Intake detector");
    rightBoxDetect.setName("Right Intake detector");

    //system checks
    systemAverageL = new AveragingQueue(AVERAGING_QUEUE_LENGTH);
    systemAverageR = new AveragingQueue(AVERAGING_QUEUE_LENGTH);

  }



  // for testing
  public double getLeftVoltage() {
    return this.leftBoxDetect.getVoltage();
  }

  public String getIntakeState() {
    return this.intakeState.toString();
  }

  /**
   * Updates the System
   * @param dt : Gives difference in time between one call to update and another
   */
  public void update(double dt) {

    switch(intakeState) {
      case STOPPED :
      if(checkBox()) {
        //if we have a box, keep it by applying a constant current
          setAmps(currentSuckIntake);
       } else {
         //otherwise don't run speed
          setSpeeds(0, 0);
       }

        break;
      case INTAKE_IN :
        //flush means both sides are the same distance from the back
        //if we don't have the box in the back of our intake
       if(!checkBox()) {

         if(rightBoxDetect.getVoltage()+0.1 > 1 && leftBoxDetect.getVoltage() < 1) {
           // if its not flush left then apply more power to the right
           setSpeeds(1 , -0.5);
         } else if (leftBoxDetect.getVoltage() > 1 && rightBoxDetect.getVoltage()+.1 < 1) {
           //if its not flush right then apply more power to the left
           setSpeeds(-0.5 , 1);
         } else {
           //else full intake speed to pull the box in
           setSpeeds(1, 1);
         }

       } else {
         //otherwise stop running it
        intakeState = States.STOPPED;
       }
        break;
        
      case AUTON_INTAKE :
        //this case is used in auto when you just need to contain the box while the arm swings down
        setSpeeds(0.35, 0.35);
        break;
      case INTAKE_OUT :

        //just spits the block out
        setSpeeds(-0.7, -0.7);
        break;
      case AUTON_OUT:
        setSpeeds(-0.45, -0.45);
        break;
      case SUPER:
        setSpeeds(-1, -1);
        break;
    }

//     SmartDashboard.putBoolean("HasBox", checkBox());
  }


  public void outakeAuto() {
    intakeState = States.AUTON_OUT;
  }
  
  public void superSpit() {
    intakeState = States.SUPER;
  }
  /**
   * Basically a feedback loop but like with current
   * @param amps : current to hold
   */
  public void setAmps(double amps) {
    intakeMotor_left.set(ControlMode.Current, amps);
    intakeMotor_right.set(ControlMode.Current, amps);
  }

  public double getAmpsLeft() {
    return intakeMotor_left.getOutputCurrent();
  }


  /**
   * Sets the intake motor speeds
   * @param left  : left motor speed
   * @param right : right motor speed
   */
  private void setSpeeds(double left, double right) {
    intakeMotor_left.set(ControlMode.PercentOutput, left);;
    intakeMotor_right.set(ControlMode.PercentOutput, right);
  }


  /**
   * Intakes the power cube
   */
  public void autonSucc() {
      intakeState = States.AUTON_INTAKE;
  }

  /**
   * Intakes the power cube
   */
  public void succ() {
      intakeState = States.INTAKE_IN;
  }

  /**
   * Spits the power cube out
   */
  public void spit() {
      intakeState = States.INTAKE_OUT;
  }

  /**
   * Stops the intake motors
   */
  public void stop() {
      intakeState = States.STOPPED;
  }


  /**
   * Checks whether there is a box using beam break
   * @return : returns whether if there is a box or not
   */
  public boolean checkBox() {
      return leftBoxDetect.getVoltage() > maxDistanceForEquality && rightBoxDetect.getVoltage()+.1 > maxDistanceForEquality;
  }


  /**
   * Just a good ol' system check
   */
  public void runSystemChecks() {
    try {

      succ();

      //left side averaging
      systemAverageL.update(intakeMotor_left.getOutputCurrent());
      double expectedDifferenceL = Math.abs(systemAverageL.getAverage() - currentOperatingIntake);

      //sends diagnostics
      SmartDashboard.putNumber("left intake difference", expectedDifferenceL);
      SmartDashboard.putNumber("left intake average", systemAverageL.getAverage());

      //right side averaging
      systemAverageR.update(intakeMotor_right.getOutputCurrent());
      double expectedDifferenceR = Math.abs(systemAverageR.getAverage() - currentOperatingIntake);

      //sends diagnostics
      SmartDashboard.putNumber("right intake difference", expectedDifferenceR);
      SmartDashboard.putNumber("right intake average", systemAverageR.getAverage());

      //make sure that its operating as expected
      if(expectedDifferenceL > LEFT_AMP_TOL && systemAverageL.getCurrentSize() > AVERAGING_QUEUE_LENGTH/2) {
        DriverStation.reportError("LEFT SIDE INTAKE CURRENT ISNT AT EXPECTED CURRENT " + systemAverageL.getAverage() +
            " DIFFERENCE " + expectedDifferenceL, false);
      }

      if(expectedDifferenceR > RIGHT_AMP_TOL && systemAverageR.getCurrentSize() > AVERAGING_QUEUE_LENGTH/2) {
        DriverStation.reportError("RIGHT SIDE INTAKE CURRENT ISNT AT EXPECTED CURRENT " + systemAverageR.getAverage() +
            " DIFFERENCE " + expectedDifferenceR, false);
      }

    } catch(Exception e) {
      DriverStation.reportError("cant communicate the intake current at this time", false);
    }
  }
}
