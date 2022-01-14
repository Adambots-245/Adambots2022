/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TimedManualTurretCommand extends CommandBase {
  /**
   * Creates a new ManualTurretCommand.
   */
  private final TurretSubsystem turretSubsystem;
  DoubleSupplier leftInput;
  DoubleSupplier rightInput;
  long startTime;
  long timeInMilliseconds;
  public TimedManualTurretCommand(TurretSubsystem turretSubsystem, DoubleSupplier leftInput, DoubleSupplier rightInput, long timeInMilliseconds) {
    this.turretSubsystem = turretSubsystem;
    this.leftInput = leftInput;
    this.rightInput = rightInput;
    this.timeInMilliseconds = timeInMilliseconds;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turretSubsystem.setSpeed(rightInput.getAsDouble()-leftInput.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turretSubsystem.setSpeed(0);
    System.out.println("timed manual turret command ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - startTime >= timeInMilliseconds;
  }
}