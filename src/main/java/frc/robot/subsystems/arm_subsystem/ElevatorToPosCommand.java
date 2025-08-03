package frc.robot.subsystems.arm_subsystem;

import edu.wpi.first.wpilibj2.command.Command;


public class ElevatorToPosCommand extends Command {
  // Class objects
  private ElevatorSubsystem elevatorSubsystem; 
  private TrapezoidalMotionProfile.MotionProfileResult trapezoidalMotionProfile;

  // Constants
  private final double MAX_VELOCITY     = 5.0;  // Max speed in inches/sec
  private final double MAX_ACCELERATION = 2.0;  // Max acceleration in inches/sec^2
  private final double MAX_DECELERATION = 1.0;  // Max deceleration in inches/sec^2
  private final double PERIOD           = 0.02; // 20ms periodic update (typical for FRC)
  private final double TOLERANCE        = 0.5;  // Tolerance for the Elevator in inches

  // Variables
  private double targetPos;      // Target position for the Elevator in inches
  private double startPos;       // The starting position for the Elevator
  private double targetDistance; // Target distance for the Elevator
  private double currentTime;    // Current time in seconds
  private double deltaPos;       // Current target position for the Elevator
  private boolean atPosition = false;

  
  /**
   * Set the target position to go to.
   * @param targetPos in inches.
   * @param elevatorSubsystem required subsystem object.
   */
  public ElevatorToPosCommand(double targetPos, ElevatorSubsystem elevatorSubsystem) {
    this.targetPos = targetPos;
    this.elevatorSubsystem = elevatorSubsystem;
    this.addRequirements(elevatorSubsystem);
  } // End ElevatorToPosCommand
  

  /**
   * Called when the command is initially scheduled.
   */ 
  @Override
  public void initialize() {
    startPos = elevatorSubsystem.getPosition(); // Get the current position of the elevator
    targetDistance = startPos - targetPos;

    // If already at position
    if (Math.abs(targetDistance) < TOLERANCE) { 
      atPosition = true;
    } 
    // If not already at position
    else {
    currentTime = 0.0; // Reset current time
    System.out.println("StartingElevatorMove");
    trapezoidalMotionProfile = TrapezoidalMotionProfile.generateProfile(MAX_DECELERATION, MAX_VELOCITY, MAX_ACCELERATION, targetDistance);
    atPosition = false;
    }
  } // End initialize

  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute() {
    // Increment the elapsed time
    currentTime += PERIOD;

    //// Determine the current phase of the motion profile
    // Acceleration phase
    if (currentTime <= trapezoidalMotionProfile.tAccel) {
        deltaPos = 0.5 * MAX_ACCELERATION * currentTime * currentTime;
    }
    // Constant velocity phase
    else if (currentTime <= trapezoidalMotionProfile.tAccel + trapezoidalMotionProfile.tConst) {
        double tConstStart = trapezoidalMotionProfile.tAccel;
        deltaPos = (0.5 * MAX_ACCELERATION * tConstStart * tConstStart) +
                          (MAX_VELOCITY * (currentTime - tConstStart));
    }
    // Deceleration phase
    else if (currentTime <= trapezoidalMotionProfile.tTotal) {
        deltaPos = (0.5 * MAX_ACCELERATION * trapezoidalMotionProfile.tAccel * trapezoidalMotionProfile.tAccel) +
                          (MAX_VELOCITY * trapezoidalMotionProfile.tConst) +
                          (MAX_VELOCITY * trapezoidalMotionProfile.tDeccel - 0.5 * MAX_DECELERATION * trapezoidalMotionProfile.tDeccel * trapezoidalMotionProfile.tDeccel);
    }
    // Motion profile complete
    else {
        deltaPos = targetPos;
        atPosition = true;
    }

    // Command the elevator subsystem to move to the target position
    if (targetDistance < 0) {
      elevatorSubsystem.setTargetPosition(startPos - deltaPos);
    }
    else {
      elevatorSubsystem.setTargetPosition(startPos + deltaPos);
    }
    if (Math.abs(elevatorSubsystem.elevatorEncoder.getPosition()-targetPos)<1.0){
      atPosition=true;
      elevatorSubsystem.setTargetPosition(targetPos);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atPosition;
  }

  public class TrapezoidalMotionProfile {
    public static MotionProfileResult generateProfile(double deMax, double vMax, double aMax, double dTarget) {
        double tAccel = vMax / aMax;
        double dAccel = 0.5 * aMax * tAccel * tAccel;

        double tDeccel = vMax / deMax;
        double dDeccel = 0.5 * deMax * tDeccel * tDeccel;

        double vPeak = vMax;
        double tConst = 0;
        double dConst = 0;

        // Check if the profile is triangular
        if (dDeccel + dAccel >= Math.abs(dTarget)) {
          vPeak = Math.sqrt((2 * Math.abs(dTarget)) / ((1 / aMax) + (1 / deMax)));;
          tAccel = vPeak / aMax;
          dAccel = 0.5 * aMax * tAccel * tAccel;
          tDeccel = vPeak / deMax;
          dDeccel = 0.5 * deMax * tDeccel * tDeccel;
      } else {
          dConst = Math.abs(dTarget) - (dAccel + dDeccel);
          tConst = dConst / vMax;
      }

        double tTotal = 2 * tAccel + tConst;

        return new MotionProfileResult(tAccel, tDeccel, tConst, tTotal, vPeak);
    }

    // Helper class to store the result
    public static class MotionProfileResult {
      public final double tAccel;
      public final double tDeccel;
      public final double tConst;
      public final double tTotal;
      public final double vPeak;

      public MotionProfileResult(double tAccel, double tDeccel, double tConst, double tTotal, double vPeak) {
          this.tAccel = tAccel;
          this.tDeccel = tDeccel;
          this.tConst = tConst;
          this.tTotal = tTotal;
          this.vPeak = vPeak;
      }
    }
  }
}