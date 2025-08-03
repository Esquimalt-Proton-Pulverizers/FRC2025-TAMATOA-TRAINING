// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm_subsystem;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * 
 */
public class ArmMoveCommand extends Command {

  // Class Variable
  private boolean atPosition;

  /**
   * 
   * @param elevatorTargetPos
   * @param elbowTargetAngle
   * @param wristTargetAngle
   * @param armSubsystem
   */
  public ArmMoveCommand(double elevatorTargetPos, double elbowTargetAngle, double wristTargetAngle,
                        ArmSubsystem armSubsystem) {
    ArmSubsystem.setArmTargetPos(elevatorTargetPos, elbowTargetAngle, wristTargetAngle);
    this.addRequirements(armSubsystem);
  } // End ArmMoveCommand
  

  /** 
   * Called when the command is initially scheduled.
   */
  @Override
  public void initialize() {
    System.out.println("Starting Arm Move.");
    
    // TODO add the call to arm move method
    atPosition = false;
  } // End initialize

  /** 
   * Called every time the scheduler runs while the command is scheduled.
   */ 
  @Override
  public void execute() {
    // TODO Check if arm subsystem is at the riht position
    // if (true){
    //   atPosition=true;
    // }
  } // End execute

  /** 
   * Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) {
  } // End end

  /** 
   * Returns true when the command should end.
   */ 
  @Override
  public boolean isFinished() {
    return atPosition;
  } // End isFinished

} // End ArmMoveCommand
