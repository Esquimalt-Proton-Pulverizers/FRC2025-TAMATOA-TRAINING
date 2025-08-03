package frc.robot.subsystems.arm_subsystem.elevator_subsystem;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;

import frc.robot.RobotContainer;
import frc.robot.subsystems.arm_subsystem.ArmSubsystem;


public class ElevatorSubsystem extends ArmSubsystem {
  // Position Constants
  public static final double LOCK_POSITION   =  0.0;
  public static final double LOW_POSITION    =  3.0; // Correct Low position value 2.0 
  public static final double LEVEL1_POSITION =  5.0;
  public static final double LEVEL2_POSITION = 16.5;
  public static final double LEVEL3_POSITION = LEVEL2_POSITION + 16.0;
  public static final double LEVEL4_POSITION = 58.0;
  public static final double NET_POSITION    = 50;
  public static final double CORAL_STATION_POSITION = 19.0;

  public static final double MIN_ELEVATION =  0.0;
  public static final double MAX_ELEVATION = 62.0;

  // Elevator Motor Config
  protected static SparkMax elevatorMotor = new SparkMax(1, MotorType.kBrushless);
  protected SparkMaxConfig elevatorConfig = new SparkMaxConfig();
  protected SparkClosedLoopController elevatorClosedLoopController = elevatorMotor.getClosedLoopController();
  protected RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  private double positionConversionFactor = 1 / 1.347;
  

  /** 
   * Configure the Elevator Motor and Encoder, and set the starting position to the "LOW_POSITION".
   */
  public ElevatorSubsystem() {
    elevatorConfig.encoder.positionConversionFactor(positionConversionFactor)
      .velocityConversionFactor(1);
      elevatorConfig.smartCurrentLimit(1,8,50);
    
    elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.1).i(0.000001).d(0.0000)
      .outputRange(-.3, .7, ClosedLoopSlot.kSlot0);

    elevatorConfig.closedLoop.maxMotion
      .maxVelocity(3000)
      .maxAcceleration(8000)
      .allowedClosedLoopError(1).positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        
    elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    elevatorClosedLoopController.setReference(LOW_POSITION, SparkMax.ControlType.kPosition);    
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
  public static void resetEncoder() {
    elevatorMotor.getEncoder().setPosition(0);
  } // End resetEncoder

  public double getPosition() {
    return elevatorEncoder.getPosition();
  } // End getPosition


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

} // End ElevatorSubsystem