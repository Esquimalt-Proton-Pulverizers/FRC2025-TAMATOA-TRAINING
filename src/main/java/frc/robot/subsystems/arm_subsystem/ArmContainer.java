// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm_subsystem;

import java.util.EnumSet;

import edu.wpi.first.wpilibj.Timer;


public class ArmContainer {
  // Class variables
  protected static Timer timer = new Timer();
  private static boolean hasBeenInitialized = false;

  // Subsystem intializations
  protected ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  protected DifferentialSubsystem differentialSubsystem = new DifferentialSubsystem();

  // Target Positions
  protected static double elevatorTargetPos = 0.0;
  protected static double elbowTargetAngle  = 0.0;
  protected static double wristTargetAngle  = 0.0;
  private static boolean atPosition         = false;

  // Enum Positions
  private EnumSet<ArmState> elevatorStates = EnumSet.noneOf(ArmState.class);
  private EnumSet<ElbowState> elbowStates = EnumSet.noneOf(ElbowState.class);
  private EnumSet<WristState> wristStates = EnumSet.noneOf(WristState.class);


  public enum ArmState {
    UNKNOWN,              // 
    SAFE_TO_DRIVE,        // All arm components safe for driving/moving the robot
    STATIONARY            // Elevator is at or near ground (e.g., for intaking)
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
  /**
   * Constructor for the ArmContainer.
   * this is a container that holds the Elevator, and differential Subsystems, but is not used to set their motion 
   * directly. Instead, it is used to keep track of the arm's state and provide some helper methods.
   * The ArmMoveCommand is used to set the target positions for the Elevator, and differential, and move them accordingly.
   * Initializes the timer for periodic updates.
   */
  public ArmContainer() {
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
   * Return if the entire ArmSubsytem, including the Elevator, Differential Elbow and Wrist has reached
   * its target positions.
   * @return true if the entire ArmSubsystem has reached their target positions, false otherwise.
   */
  public boolean atPosition(){
    return atPosition;
  } // End atPosition

  public static ArmPosition getArmTargetPos() {
    return new ArmPosition(wristTargetAngle, elevatorTargetPos, elbowTargetAngle);
  } // End getElevatorTargetPos
  
  /**
   * 
   * @return ArmState indicating the current state of the ArmSubsystem.
   *         Returns SAFE_TO_DRIVE if the arm is in a safe position to drive.
   */
  public ArmState getArmState() {
    // if (ele)

    // TODO finish method

    return ArmState.SAFE_TO_DRIVE;
  }

  /** 
   * Called once per scheduler run. Only if added to Robot.java
   */
  public void periodic() {
    //TODO Add state checking code here

    // Only run once everytime the timer has elapsed.
    if(timer.hasElapsed(2.0)) {
      System.out.println("-------------------- ArmSubsystem --------------------");
      timer.reset();
    }
  } // End periodic

  /**Helper class to store the Arm Positions*/
  public static class ArmPosition {
    public final double wristAngle;
    public final double elbowAngle;
    public final double elevatorHeightIn;
  /**
   * Constructor for the ArmPosition class.
   * @param elevatorHeightIn Target position for the Elevator in inches.
   * @param elbowAngle Target Angle for the Elbow.  
   * @param wristAngle Target Angle for the Wrist.
   */

    public ArmPosition(double elevatorHeightIn, double elbowAngle, double wristAngle) {
        this.wristAngle = wristAngle;
        this.elbowAngle = elbowAngle;
        this.elevatorHeightIn = elevatorHeightIn;
    }
  }

} // End ArmSubsystem
