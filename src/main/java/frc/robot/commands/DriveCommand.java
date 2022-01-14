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
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetEncoders();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("FBI: " + forwardBackwardInput.getAsDouble() + " - RI:" + rotationInput.getAsDouble());
    // rotationInput.getAsDouble());
    drivetrain.getAverageDriveEncoderValue();
    // System.out.println("Rotation: " + rotationInput.getAsDouble());
    drivetrain.arcadeDrive(forwardBackwardInput.getAsDouble(), rotationInput.getAsDouble());
    SmartDashboard.putNumber("driveencoder", drivetrain.getAverageDriveEncoderValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("DriveCommand interrupted");
    } else {
      drivetrain.arcadeDrive(0, 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
