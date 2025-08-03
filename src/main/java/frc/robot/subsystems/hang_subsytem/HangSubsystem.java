// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hang_subsytem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HangSubsystem extends SubsystemBase {
  
  // Constants 
  private final int WINCH_MOTOR_CAN_ID = 10;
  private final int INTAKE_MOTOR_CANID = 11;
  private final int LATCH_SERVO_PORT = 0;
  private final double WINCH_RETRACT_VOLTAGE = -5;

  private final double INTAKE_VOLTAGE = 6.0;
  private final double HOLDING_VOLTAGE = 0.0;
  private final double WINCH_RETRACTED_POS = 0.0;
  private final double WINCH_EXTENDED_POS = 690.0;

  public enum LatchServoPosition {
    LATCHED(1),
    FREE(0.5);

    final double value;

    private LatchServoPosition(double value) {
      this.value = value;
    }
  } // end latchServoPosition

  // Hardware
  private SparkMax winchMotor;
  private SparkClosedLoopController winchController;
  private RelativeEncoder winchEncoder; 
  private Servo latchServo;
  private SparkMax intakeMotor;
  private SparkClosedLoopController intakeController;


  public HangSubsystem() {
    winchMotor = new SparkMax(WINCH_MOTOR_CAN_ID, MotorType.kBrushless);
    winchEncoder = winchMotor.getEncoder();
    winchController = winchMotor.getClosedLoopController();
    intakeMotor = new SparkMax(INTAKE_MOTOR_CANID, MotorType.kBrushless);
    intakeController = intakeMotor.getClosedLoopController();
    latchServo = new Servo(LATCH_SERVO_PORT);

    configureWinchMotor();
    configureIntakeMotor();
    setLatchServoPosition(LatchServoPosition.FREE);
    
    intakeController.setReference(0, SparkMax.ControlType.kVoltage);
    intakeMotor.getEncoder();
  } // end constructor
      
  private void configureIntakeMotor() {
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }
  
  private void configureWinchMotor() {
    SparkMaxConfig winchConfig = new SparkMaxConfig();
    winchConfig
      .inverted(true)
      .smartCurrentLimit(1, 8, 50);

    winchConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(0.1, 0.0, 0.0)
      .outputRange(-1, 1, ClosedLoopSlot.kSlot0);

    winchMotor.configure(winchConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  } // end configureWinchMotor
  
  /**shows the latching servo position on the dashboard */
  @Override public void periodic() {
    SmartDashboard.putNumber("Servo Position", latchServo.getPosition());
  }

  private void setLatchServoPosition(LatchServoPosition position) {
    latchServo.set(position.value);
  }

  protected void setWinchPosition(double position) {
      winchController.setReference(position, ControlType.kPosition);
  }

  public double getWinchPosition() {
      return winchEncoder.getPosition();
  }

  // Commands
  /** Sequence to unlatch and reel out the winch  to the EXTENDED_POSITION*/
  public SequentialCommandGroup extend() {
    return new SequentialCommandGroup(
      release(),
      new WinchToPositionCommand(this, getWinchPosition()-3), //TODO Test if this line works as volatile for the getwinchposition
      new WinchToPositionCommand(this, WINCH_EXTENDED_POS)
    );
  } // end extend()

  /** Sequence to latch and reel in the winch */
  public SequentialCommandGroup retract() {
    return new SequentialCommandGroup(
      latch(),
      new WinchToPositionCommand(this, WINCH_RETRACTED_POS)  
    );
  } // end retract()
  
  /** Sets the Winch Motor to its WINCH_RETRACT_VOLTAGE and latches the servo runs instantly*/
  public Command manualRetract() {
    return Commands.runOnce(() -> {
      winchController.setReference(WINCH_RETRACT_VOLTAGE, ControlType.kVoltage);
      setLatchServoPosition(LatchServoPosition.LATCHED);
    });    
  }
  /** Turns off the winch motor and resets the encoder position, leaving the latching servo
   * in whatever position it was in. runs instantly
   */
  public Command resetWinch() {
    return Commands.runOnce(() -> {
      winchController.setReference(0, ControlType.kVoltage);
      winchMotor.getEncoder().setPosition(0);
    });
  }
  
  /**Turns the Cage Intake motor on at the set INTAKE_VOLTAGE */
  public Command intake(){
    return Commands.runOnce(() -> intakeController.setReference(INTAKE_VOLTAGE, ControlType.kVoltage));    
  }
 /**stops the Cage Intake motor */
  public Command stopIntake(){
    return Commands.runOnce(() -> intakeController.setReference(HOLDING_VOLTAGE, ControlType.kVoltage));    
  }
  //Servo Commands so they could be added to a sequential command
  private Command release() {
    return Commands.runOnce(() -> setLatchServoPosition(LatchServoPosition.FREE));    
  }

  private Command latch() {
    return Commands.runOnce(() -> setLatchServoPosition(LatchServoPosition.LATCHED));    
  }

}
