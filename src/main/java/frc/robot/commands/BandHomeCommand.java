/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatapultSubsystem;

public class BandHomeCommand extends CommandBase {
  /**
   * Creates a new Command for testing.
   */

  private CatapultSubsystem catapultSubsystem;
  private Double offset;

  public BandHomeCommand(CatapultSubsystem catapultSubsystem, double offset) {
    this.catapultSubsystem = catapultSubsystem;
    this.offset = offset;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(catapultSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    catapultSubsystem.setEncoderMode(false);
    catapultSubsystem.runBandMotor(0.7);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      catapultSubsystem.setBandTarget(offset*4096*20);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return catapultSubsystem.getBandSwitch();
  }
}
