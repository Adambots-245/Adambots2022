
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HangSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class UnsuppressClampingCommand extends CommandBase {

  private final HangSubsystem hangSubsystem;

  public UnsuppressClampingCommand(HangSubsystem hangSubsystem) {
    this.hangSubsystem = hangSubsystem;
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
    hangSubsystem.unsuppressClamp();
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