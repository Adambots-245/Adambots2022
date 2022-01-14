/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class StartIntakeCommand extends CommandBase {
  /**
   * Creates a new IntakeCommand.
   */
  private final IntakeSubsystem intakeSubsystem;
  private DoubleSupplier speedInput;

  public StartIntakeCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier speedInput) {
    this.intakeSubsystem = intakeSubsystem;
    this.speedInput = speedInput;
    addRequirements(intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.intake(speedInput.getAsDouble());
   // System.out.println("intake speed: " + speedInput.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.intake(0);
    if (interrupted) {
      System.out.println("StartIntakeCommand interrupted");
    }
    else
    {
      System.out.println("StartIntakeCommand Ended");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
