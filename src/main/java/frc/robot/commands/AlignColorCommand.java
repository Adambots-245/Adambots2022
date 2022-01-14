/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.subsystems.ControlPanelSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class AlignColorCommand extends CommandBase {
  private final ControlPanelSubsystem controlPanel;

  public AlignColorCommand(ControlPanelSubsystem controlPanel) {
    this.controlPanel = controlPanel;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(controlPanel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      controlPanel.startAligner(ControlPanelSubsystem.Modes.Alignment);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      controlPanel.monitorAligner();
      controlPanel.putDashAligner();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  //Runs until interrupted
  @Override
  public boolean isFinished() {
    return controlPanel.isFinished(ControlPanelSubsystem.Modes.Alignment);
  }
}
