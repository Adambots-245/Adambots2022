/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

public class PanelMotor extends CommandBase {
  /**
   * Creates a new PanelMotor.
   */
  private final ControlPanelSubsystem controlPanelSubsystem;
  public PanelMotor(ControlPanelSubsystem controlPanelSubsystem) {
    this.controlPanelSubsystem = controlPanelSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(controlPanelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controlPanelSubsystem.startMotor(ControlPanelSubsystem.Modes.Rotations);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controlPanelSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
