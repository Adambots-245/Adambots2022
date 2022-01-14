/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GondolaSubsystem;
import frc.robot.subsystems.HangSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class GondolaCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new HangCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DoubleSupplier gondolaSpeed;
  private final GondolaSubsystem gondolaSubsystem;
  
  public GondolaCommand(GondolaSubsystem gondolaSubsystem, DoubleSupplier speed) {
    this.gondolaSubsystem = gondolaSubsystem;
    gondolaSpeed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gondolaSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gondolaSubsystem.gondola(gondolaSpeed.getAsDouble());
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("GondolaCommand ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
