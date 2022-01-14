/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BlasterSubsystem;

public class BlasterPercentOutput extends CommandBase {
  /**
   * Creates a new BlasterPercentOutput.
   */
  BlasterSubsystem blasterSubsystem;
  DoubleSupplier speedInput;
  public BlasterPercentOutput(BlasterSubsystem blasterSubsystem, DoubleSupplier speedInput) {
    this.blasterSubsystem = blasterSubsystem;
    this.speedInput = speedInput;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(blasterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    blasterSubsystem.output(speedInput.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    blasterSubsystem.output(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
