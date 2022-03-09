/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Gamepad.Buttons;
import frc.robot.subsystems.CatapultSubsystem;

public class BandHomeCommand extends CommandBase {
  /**
   * Creates a new Command for testing.
   */

  private CatapultSubsystem catapultSubsystem;
  private Double offset;
  private Boolean SecondControl;
  private Boolean abort;

  public BandHomeCommand(CatapultSubsystem catapultSubsystem, double offset, Boolean SecondControl) {
    this.catapultSubsystem = catapultSubsystem;
    this.offset = offset;
    this.SecondControl = SecondControl;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(catapultSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if (SecondControl) { //Only run if both bumpers are pressed, otherwise, cancel the command
      catapultSubsystem.setEncoderMode(false);
      catapultSubsystem.runBandMotor(0.5);
      abort = false;
    // }
    // else {
      // abort = true;
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted && !abort) {
      catapultSubsystem.setBandTarget(offset*4096*20);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return catapultSubsystem.getBandSwitch() || abort;
  }
}
