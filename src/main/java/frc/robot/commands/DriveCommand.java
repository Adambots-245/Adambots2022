/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.utils.Log;

public class DriveCommand extends CommandBase {
  public final DriveTrainSubsystem drivetrain;
  public final DoubleSupplier forwardBackwardInput;
  public final DoubleSupplier rotationInput;

  /**
   * Creates a new DriveCommand.
   */
  public DriveCommand(DriveTrainSubsystem inputDriveTrain, DoubleSupplier straightInput, DoubleSupplier turnInput) {
    drivetrain = inputDriveTrain;
    forwardBackwardInput = straightInput;
    rotationInput = turnInput;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // drivetrain.resetEncoders();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drivetrain.getAverageDriveEncoderValue();
    drivetrain.arcadeDrive(forwardBackwardInput.getAsDouble(), rotationInput.getAsDouble());
    SmartDashboard.putNumber("driveencoder", drivetrain.getAverageDriveEncoderValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Log.info("DriveCommand interrupted");
    } else {
      drivetrain.arcadeDrive(0, 0);
    }
  }

  // As long as this command is used as a default command it should never end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
