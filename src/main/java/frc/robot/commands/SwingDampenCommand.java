/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.PitchPIDSubsystem;

public class SwingDampenCommand extends CommandBase {
  /**
   * Creates a new Command for testing.
   */

  private final DriveTrainSubsystem driveTrainSubsystem;
  private final PitchPIDSubsystem pitchPIDSubsystem;

  public SwingDampenCommand(DriveTrainSubsystem driveTrainSubsystem, PitchPIDSubsystem pitchPIDSubsystem) {
    this.driveTrainSubsystem = driveTrainSubsystem;
    this.pitchPIDSubsystem = pitchPIDSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(driveTrainSubsystem);
    addRequirements(pitchPIDSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // driveTrainSubsystem.arcadeDrive(pitchPIDSubsystem.getOutput()*10, 0);

    if (RobotMap.Accelerometer.getZ() > 0)
      driveTrainSubsystem.arcadeDrive(-1, 0);
    else
      driveTrainSubsystem.arcadeDrive(1, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
