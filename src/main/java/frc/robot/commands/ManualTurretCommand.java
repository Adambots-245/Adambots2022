/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.Log;

public class ManualTurretCommand extends CommandBase {
  /**
   * Creates a new ManualTurretCommand.
   */
  private final TurretSubsystem turretSubsystem;
  DoubleSupplier leftInput;
  DoubleSupplier rightInput;
  public ManualTurretCommand(TurretSubsystem turretSubsystem, DoubleSupplier leftInput, DoubleSupplier rightInput) {
    this.turretSubsystem = turretSubsystem;
    this.leftInput = leftInput;
    this.rightInput = rightInput;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    //Log.infoF("Ending Manual Turrent Command. Interrupted = %b", interrupted);
    System.out.println("manual turret command ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
