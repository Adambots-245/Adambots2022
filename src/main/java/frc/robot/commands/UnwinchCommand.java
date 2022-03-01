
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HangSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class UnwinchCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  /**
   * Creates a new HangCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DoubleSupplier winchSpeed;
  private final HangSubsystem hangSubsystem;

  public UnwinchCommand(HangSubsystem hangSubsystem) {
    this.hangSubsystem = hangSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hangSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        hangSubsystem.winchUp();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hangSubsystem.winchUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hangSubsystem.winchOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}