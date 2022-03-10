/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatapultSubsystem;

public class BandMoveCommand extends CommandBase {
  /**
   * Creates a new Command for testing.
   */

  private final CatapultSubsystem catapultSubsystem;
  private final double pos;

  public BandMoveCommand(CatapultSubsystem catapultSubsystem, double pos) {
    this.catapultSubsystem = catapultSubsystem;
    this.pos = pos;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(catapultSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    catapultSubsystem.setEncoderMode(true);
    catapultSubsystem.setBandTarget(pos*4096*20); //4096 ticks per rev, 20 revs per inch
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
