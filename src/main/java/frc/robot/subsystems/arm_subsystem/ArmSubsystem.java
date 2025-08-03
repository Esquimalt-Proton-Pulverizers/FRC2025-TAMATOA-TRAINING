// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm_subsystem;

import java.util.EnumSet;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm_subsystem.differential_subsystem.DifferentialSubsystem;
import frc.robot.subsystems.arm_subsystem.elevator_subsystem.ElevatorSubsystem;
import frc.robot.subsystems.arm_subsystem.intake_subsystem.IntakeSubsystem;


public class ArmSubsystem extends SubsystemBase {
  // Class variables
  protected static Timer timer = new Timer();
  private static boolean hasBeenInitialized = false;

  // Subsytem intializations
  protected ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  protected DifferentialSubsystem differentialSubsystem = new DifferentialSubsystem();

  // Target Positions
  protected static double elevatorTargetPos = 0.0;
  protected static double elbowTargetAngle  = 0.0;
  protected static double wristTargetAngle  = 0.0;
  private static boolean atPosition         = false;

  // Enum Positions
  private EnumSet<ElevatorState> elevatorStates = EnumSet.noneOf(ElevatorState.class);
  private EnumSet<ElbowState> elbowStates = EnumSet.noneOf(ElbowState.class);
  private EnumSet<WristState> wristStates = EnumSet.noneOf(WristState.class);


  public enum ElevatorState {
    UNKNOWN,              // Elevator is below 20"
    SAFE_TO_DRIVE,        // Elevator position is safe for driving/moving the robot
    SAFE_FOR_ELBOW_MOVE,  // Elevator is in a position where it's safe for elbow to move
    AT_GROUND_POSITION    // Elevator is at or near ground (e.g., for intaking)
  } // End ElevatorState

  public enum ElbowState {
    SAFE_TO_MOVE_ELEVATOR_ABOVE_30,  // Elbow is in a position where elevator can go above 20"
    SAFE_TO_GO_TO_GROUND,            // Elbow is safe to lower to ground
    SAFE_TO_DRIVE,                   // Elbow is in a safe driving position
    IN_SCORING_POSITION,             // Elbow is positioned for scoring
    LOCKED_FOR_SAFETY                // Elbow movement is restricted for safety
  } // End ElbowState

  public enum WristState {
    SAFE_WITH_ELBOW_FOR_GROUND,      // Wrist is in a position safe when elbow is above a certain angle
    SAFE_FOR_ELBOW_ABOVE_THRESHOLD,  // Wrist is safe when elbow is below a certain angle
    PARALLEL_TO_GROUND,              // Wrist is parallel to ground
    PERPENDICULAR_TO_ARM,            // Wrist is aligned with arm for certain tasks
    STOWED_POSITION,                 // Wrist is in a stowed position
    UNSAFE_POSITION_WARNING          // Wrist is in an unsafe config and needs to be corrected
  } // End WristState


  public ArmSubsystem() {
    timer.start();
  } // End ArmSubsystem

  /**
   * On robot's first initialization, intialize the ArmSubsystem, calling the intialize method 
   * for the Elevator, Elbow, Wrist and Intake Subsystem.
   * 
   * If not the first initialization, do nothing.
   */
  public static void initialize(){
    if(!hasBeenInitialized) {
        ElevatorSubsystem.initialize();
        DifferentialSubsystem.initialize();
        IntakeSubsystem.initialize();
        hasBeenInitialized = true;
    }
  }


  /**
   * Set the target position for the ArmSubsytem.
   * @param setElevatorTargetPos Target position for the Elevator in inches.
   * @param setElbowTargetAngle Target Angle for the Elbow.
   * @param setWristTargetAngle Target Angle for the Wrist.
   */
  protected static void setArmTargetPos(double setElevatorTargetPos, double setElbowTargetAngle, double setWristTargetAngle) {
    elevatorTargetPos = setElevatorTargetPos;
    elbowTargetAngle  = setElbowTargetAngle;
    wristTargetAngle  = setWristTargetAngle;
    atPosition        = false;
  } // End setArmTargetPos

  /**
   * Set the target position for the ArmSubsytem (Command based.)
   * @param setElevatorTargetPos Target position for the Elevator in inches.
   * @param setElbowTargetAngle Target Angle for the Elbow.
   * @param setWristTargetAngle Target Angle for the Wrist.
   */
  public Command setArmTargetPositionCommand(double setElevatorTargetPos, double setElbowTargetAngle, double setWristTargetAngle) {
      return Commands.runOnce(() -> {setArmTargetPos(setElevatorTargetPos, setElbowTargetAngle, setWristTargetAngle);});    
  } // End setArmTargetPositionCommand


  /**
   * Return if the entire ArmSubsytem, including the Elevator, Differential Elbow and Wrist has reached
   * its target positions.
   * @return true if the entire ArmSubsystem has reached their target positions, false otherwise.
   */
  public boolean atPosition(){
    return atPosition;
  } // End atPosition

  public static double getElevatorTargetPos() {
    return elevatorTargetPos;
  } // End getElevatorTargetPos
  public static double getElbowTargetAngle() {
    return elbowTargetAngle;
  } // End getElbowTargetAngle
  public static double getWristTargetAngle() {
    return wristTargetAngle;
  } // End getWristTargetAngle
  public static void setElevatorTargetPos(double targetPos) {
    elevatorTargetPos = targetPos;
  } // End setElevatorTargetPos
  public static void setElbowTargetAngle(double targetAngle) {
    elbowTargetAngle = targetAngle;
  } // End setElbowTargetAngle
  public static void setWristTargetAngle(double targetAngle) {
    wristTargetAngle = targetAngle;
  } // End setWristTargetAngle


  /**
   * 
   * @return
   */
  private ElevatorState getElevatorState() {
    // if (ele)

    // TODO finish method

    return ElevatorState.AT_GROUND_POSITION;
  }

  /** 
   * Called once per scheduler run.
   */
  @Override
  public void periodic() {

    // Only run once everytime the timer has elapsed.
    if(timer.hasElapsed(2.0)) {
      System.out.println("-------------------- ArmSubsystem --------------------");
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

} // End ArmSubsystem
