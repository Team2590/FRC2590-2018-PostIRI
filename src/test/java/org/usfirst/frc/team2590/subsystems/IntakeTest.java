package org.usfirst.frc.team2590.subsystems;

import java.util.List;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.junit.Assert;
import org.junit.runner.RunWith;
import org.mockito.ArgumentCaptor;
import org.mockito.Mock;
import org.mockito.Mockito;
//import org.mockito.MockitoAnnotations;
//import org.mockito.exceptions.misusing.UnnecessaryStubbingException;
import org.mockito.junit.MockitoJUnitRunner;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;

@RunWith(MockitoJUnitRunner.class)
public class IntakeTest {

    @Mock
    List<String> mockedList;
         
    @Before
    public void init() {
    }

    @After
    public void teardown() {
    }
    Intake intake = new Intake();

    @Mock
    VictorSPX leftMotorMock, rightMotorMock;
    
    @Mock
    AnalogInput leftBox, rightBox;


    /**
     *  The following test emulates testing the left and right box sensor voltage to see whether we 
     *  should turn on motors and pull ("succ") the box in.
     */
    @Test
    public void testIntakeBoxFound() {
        Assert.assertEquals(1, 1);
        // These lines tell our 'mock sensors' what values to return when getVoltage() is called on
        // them by our Intake class in order to exercise the logic we're targeting in our test.
        Mockito.when(leftBox.getVoltage()).thenReturn(2.0);
        Mockito.when(rightBox.getVoltage()).thenReturn(2.0);

       // Initialze the Intake class that we're testing with mock versions of the motors/sensors that 
       // we can measure/control, instead of the actual versions that interact with robot hardware.
       intake.init(leftMotorMock, rightMotorMock, leftBox, rightBox);
       intake.configure();

       // Output current intake state (STOPPED=default), and call the method succ() that sets the state 
       // of the intake to sense/pull in a box (game piece) if one is detected.  In real life, succ()
       // is invoked when a button is pressed on the joystick. 
       System.out.println("Intake state 1: " + intake.getIntakeState());
       intake.succ();

       // Now that we've initiated detection mode (INTAKE_IN), we are going to call the update() method
       // which will check the sensors and adjust the motors accordingly.  Our mock sensors above are already
       // programmed to return voltage of 2.0 so the code logic should act as if a box was found.  Note under
       // normal operating conditions, update() is called repeatedly many times per second.
       System.out.println("Intake state 2: " + intake.getIntakeState());
       intake.update(0.1);

       // Output current state.. if box is found the code should update the status to 'stopped'
       // and the motors should be set to a constant 'suck' speed to hold onto the box.
       System.out.println("Intake state 3: " + intake.getIntakeState());
       Assert.assertEquals(intake.getIntakeState().toString(), "STOPPED");

       // Now verify that once we've 'stopped' the intake, the intake class has properly
       // set the output current to a constant value of 2.0 to hold onto the box until we release it.
       // call our 'update' method which should then make an assessment of whether we have a box,
       // and if so, hold it (by keeping steady current), or let it go (by setting current to 0).
       // in this case, we haven't changed our mocks so the sensors will indicate that we still
       // have the box so we should maintain steady current.
       intake.update(0.1);
       ArgumentCaptor<ControlMode> argument1 = ArgumentCaptor.forClass(ControlMode.class);
       ArgumentCaptor<Double> argument2 = ArgumentCaptor.forClass(Double.class);
       Mockito.verify(leftMotorMock, Mockito.times(1)).set(argument1.capture(), argument2.capture());
       System.out.println("After acquiring box, left motor holding current: " + argument2.getValue());
       Assert.assertEquals((Double)2.0, argument2.getValue());
       System.out.println("Intake state 4: " + intake.getIntakeState());
       Assert.assertEquals(intake.getIntakeState().toString(), "STOPPED");

       
       // Now, act as if we've released the box.
       // this is done by setting the box sensors to return 0 indicating no box found.  Then call our update() method
       // and verify that we will indeed drop current to zero.  
       intake.spit();
       intake.update(0.1);
       Mockito.verify(leftMotorMock, Mockito.times(2)).set(argument1.capture(), argument2.capture());
       System.out.println("After spitting out box, left motor value: " + argument2.getValue());
       Assert.assertEquals((Double)(-0.7), argument2.getValue());
       System.out.println("Intake state 5: " + intake.getIntakeState());
 
       // Now do the same but for the right motor.  should be the same.
       Mockito.verify(rightMotorMock, Mockito.times(2)).set(argument1.capture(), argument2.capture());
       System.out.println("After spitting out box, right motor value: " + argument2.getValue());
       Assert.assertEquals((Double)(-0.7), argument2.getValue());

       // Now, indicate that we truly have dropped the box by setting the sensor value to return 0.
       Mockito.when(leftBox.getVoltage()).thenReturn(0.0);
       //Mockito.when(rightBox.getVoltage()).thenReturn(0.0); // unnecessary stubbing -- never actually referenced if leftbox is zero in Intake.checkBox() so the framework will by defult throw an exception.  Try it..
       intake.stop();
       intake.update(0.1);
       System.out.println("Intake state 6: " + intake.getIntakeState());
       Mockito.verify(leftMotorMock, Mockito.times(3)).set(argument1.capture(), argument2.capture());
       System.out.println("After dropping, the intake class set the left motor current to: " + argument2.getValue());
       Assert.assertEquals((Double)0.0, argument2.getValue());
 
    }

    @Test
    public void testIntakeNoBoxFound() {
     
        Mockito.when(leftBox.getVoltage()).thenReturn(0.0);
        Mockito.when(rightBox.getVoltage()).thenReturn(0.0);
   
       intake.init(leftMotorMock, rightMotorMock, leftBox, rightBox);
       intake.configure();

       System.out.println("Intake state 1: " + intake.getIntakeState());
       intake.succ();
       System.out.println("Intake state 2: " + intake.getIntakeState());
       intake.update(0.1);
       System.out.println("Intake state 3: " + intake.getIntakeState());

       Assert.assertEquals(intake.getIntakeState().toString(), "INTAKE_IN");
    }
    @Test
    public void testAutonIntake() {

        ArgumentCaptor<ControlMode> argument1 = ArgumentCaptor.forClass(ControlMode.class);
        ArgumentCaptor<Double> argument2 = ArgumentCaptor.forClass(Double.class);

        intake.init(leftMotorMock, rightMotorMock, leftBox, rightBox);
        intake.autonSucc();

        System.out.println("Intake state: " + intake.getIntakeState());
      // Now do the same but for the right motor.  should be the same.
      Mockito.verify(leftMotorMock, Mockito.times(1)).set(argument1.capture(), argument2.capture());
      System.out.println("After spitting out box, right motor value: " + argument2.getValue());
      Assert.assertEquals((Double)(0.0), argument2.getValue());
      intake.update(0.1);
      Mockito.verify(leftMotorMock, Mockito.times(2)).set(argument1.capture(), argument2.capture());
      System.out.println("After update, right motor value: " + argument2.getValue());
      Assert.assertEquals((Double)0.35, argument2.getValue());

    }
   /*  @Test
    public void testAdjustSpeedLeft() {
        Mockito.when(leftBox.getVoltage()).thenReturn(1.0);
        Mockito.when(rightBox.getVoltage()).thenReturn(0.4);

        //Mockito.verify(leftMotorMock).set()
        // intakeMotor_left.set
    } */

}