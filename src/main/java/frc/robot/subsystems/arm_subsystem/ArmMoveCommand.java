// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm_subsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm_subsystem.ArmContainer.ArmPosition;

/**
 * 
 */
public class ArmMoveCommand extends Command {

  // Class Variable
  private boolean atPosition;
  private ArmPosition armTargetPosition;
  private ArmPosition currentArmPosition;

  /**
   * 
   * @param armTargetPosition the target position for the arm to move to
   * @param armContainer the ArmContainer that holds the arm subsystems 
   * This command is used to move the arm to a specific position.
   */
  public ArmMoveCommand(ArmPosition armTargetPosition,
                        ArmContainer armContainer) {
    this.armTargetPosition = armTargetPosition;
    this.atPosition = false;
    //the armContainer only is used to keep track of the arm subsystem and its components
    //the required subsytems to actually move are the ones inside the armContainer
    this.addRequirements(armContainer.elevatorSubsystem, armContainer.differentialSubsystem);
  } // End ArmMoveCommand
  

  /** 
   * Called when the command is initially scheduled.
   */
  @Override
  public void initialize() {
    System.out.println("Starting Arm Move.");
    atPosition = false;
  } // End initialize

  /** 
   * Called every time the scheduler runs while the command is scheduled.
   */ 
  @Override
  public void execute() {
    // TODO check arm states and give target positions to the relevant subsystems as required
    // TODO Check if arm subsystem is at the right position
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
