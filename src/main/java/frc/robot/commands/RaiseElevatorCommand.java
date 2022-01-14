/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.HangSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class RaiseElevatorCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new HangCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private DoubleSupplier elevateSpeed;
  private boolean overrideFlag;
  private final HangSubsystem hangSubsystem;
  
  public RaiseElevatorCommand(HangSubsystem subsystem, DoubleSupplier elevateSpeed, boolean overrideFlag) {
    this.hangSubsystem = subsystem;
    this.elevateSpeed = elevateSpeed;
    this.overrideFlag = overrideFlag;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hangSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
    hangSubsystem.climb(elevateSpeed.getAsDouble(), overrideFlag);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("raiseElevator ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
