package frc.robot.subsystems.arm_subsystem;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;

import frc.robot.RobotContainer;


public class ElevatorSubsystem extends SubsystemBase {
  // Position Constants
  public static final double LOCK_POSITION   =  0.0;
  public static final double LOW_POSITION    =  3.0; // Correct Low position value 2.0 
  public static final double LEVEL1_POSITION =  5.0;
  public static final double LEVEL2_POSITION = 16.5;
  public static final double LEVEL3_POSITION = LEVEL2_POSITION + 16.0;
  public static final double LEVEL4_POSITION = 58.0;
  public static final double NET_POSITION    = 50;
  public static final double CORAL_STATION_POSITION = 19.0;

  private static final double MIN_ELEVATION =  0.0;
  private static final double MAX_ELEVATION = 62.0;
  private static final double POSITION_CONVERSION_FACTOR = 1 / 1.347;

  // Elevator Motor Config
  private static final int ELEVATOR_CAN_ID = 1;
  private static SparkMax elevatorMotor = new SparkMax(ELEVATOR_CAN_ID, MotorType.kBrushless);
  private SparkMaxConfig elevatorConfig = new SparkMaxConfig();
  private SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();
  protected RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  
  //Subsystem variables and states
  private static double elevatorTargetPos = 0;
  private static Timer timer = new Timer();

  /** 
   * Configure the Elevator Motor and Encoder, and set motor output to zero on startup".
   */
  protected ElevatorSubsystem() {
    elevatorConfig.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR)
      .velocityConversionFactor(1);
      elevatorConfig.smartCurrentLimit(1,8,50);
    
    elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1).i(0.000000).d(0.0000)
      .outputRange(-.3, .7, ClosedLoopSlot.kSlot0);

    elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    elevatorClosedLoopController.setReference(0, SparkMax.ControlType.kVoltage);    
    timer.start();
  } // End ElevatorSubsystem

  /**
   * Set and go to the target position.
   * @param targetPosition of the Elevator in inches.
   */
  protected void setTargetPosition(double targetPosition){
    elevatorTargetPos = targetPosition;
    elevatorClosedLoopController.setReference(elevatorTargetPos, ControlType.kPosition);
  } // End setTargetPosition

  /**
   * Reset the encoder value of the Elevator Motor.
   */
  protected static void resetEncoder() {
    elevatorMotor.getEncoder().setPosition(0);
  } // End resetEncoder

  public double getPosition() {
    return elevatorEncoder.getPosition();
  } // End getPosition

  //TODO I don't like this being public and bypassing all the other system safety measures. A homing command is the better solution
  /**
   * Manual movement of the Elevator. Moves by the set distance increments, until the 
   * min/ max positions has been reached, unless Manual Override is active, in which 
   * case the Elevator can move beyond those limits.
   * @param distanceIncrement
   */
  public void manualMove(double distanceIncrement){
    double newTarget = distanceIncrement + elevatorTargetPos;
    setTargetPosition(newTarget);

    if ((newTarget >= MIN_ELEVATION) && (newTarget <= MAX_ELEVATION)) {
      setTargetPosition(newTarget);
    } else if (RobotContainer.manualOverrideToggle) {
      setTargetPosition(newTarget);
    }
  } // End manualMove


  /** 
   * Called once per scheduler run.
   */
  @Override
  public void periodic() {
    // Put code here to be run every loop
    if(timer.hasElapsed(2.0)) {
      System.out.println("Elevator target position" + elevatorTargetPos);
      System.out.println("Elevator Level: " + getPosition());
      timer.reset();
    }
  } // End periodic

  /** 
   * Called once per scheduler run within a simulation.
   */
  @Override
  public void simulationPeriodic() {
    periodic();
  } // End simulationPeriodic

  public static void initialize() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'initialize'");
  }

} // End ElevatorSubsystem