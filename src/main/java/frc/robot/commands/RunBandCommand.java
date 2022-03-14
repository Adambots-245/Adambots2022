// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatapultSubsystem;
import frc.robot.utils.Log;

public class RunBandCommand extends CommandBase {
  private CatapultSubsystem catapultSubsystem;
  private Double speed;

  /** Creates a new RunBandCommand. */
  public RunBandCommand(CatapultSubsystem catapultSubsystem, Double speed) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.catapultSubsystem = catapultSubsystem;
    this.speed = speed;
    addRequirements(catapultSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("Band Motor Command ..." + speed.getAsDouble());
    Log.infoF("Band Motor Command: %f", speed);
    catapultSubsystem.setEncoderMode(false);
    catapultSubsystem.runBandMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    catapultSubsystem.runBandMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
