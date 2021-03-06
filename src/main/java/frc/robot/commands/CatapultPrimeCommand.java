/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatapultSubsystem;

public class CatapultPrimeCommand extends CommandBase {
  /**
   * Creates a new Command for testing.
   */

  private final CatapultSubsystem catapultSubsystem;

  public CatapultPrimeCommand(CatapultSubsystem catapultSubsystem) {
    this.catapultSubsystem = catapultSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(catapultSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    catapultSubsystem.runCatapult(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Running");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    catapultSubsystem.runCatapult(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return catapultSubsystem.getCatapultSwitch();
  }
}
