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

public class TurretManualCommand extends CommandBase {
  private TurretSubsystem turretSubsystem;
  private DoubleSupplier leftInput;
  private DoubleSupplier rightInput;
  /**
   * Creates a new TurretManualCommand.
   */
  public TurretManualCommand(TurretSubsystem turretSubsystem, DoubleSupplier leftInput, DoubleSupplier rightInput) {
    this.turretSubsystem = turretSubsystem;
    this.leftInput = leftInput;
    this.rightInput = rightInput; 
    addRequirements(turretSubsystem); 

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turretSubsystem.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turretSubsystem.changeOffset(rightInput.getAsDouble() - leftInput.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("TurretManualCommand interrupted");
    }
    else
    {
      System.out.println("TurretManualCommand Ended");
    }
    turretSubsystem.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
