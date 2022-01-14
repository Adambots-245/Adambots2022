/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;

public class ConveyorCommand extends CommandBase {
  /**
   * Creates a new ConveyorCommand.
   */
  private final ConveyorSubsystem conveyorSubsystem;
  private final DoubleSupplier speedInput;

  public ConveyorCommand(ConveyorSubsystem conveyorSubsystem, DoubleSupplier speedInput) {
    this.conveyorSubsystem = conveyorSubsystem;
    this.speedInput = speedInput;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conveyorSubsystem.runConveyor(speedInput.getAsDouble(), false);
    conveyorSubsystem.runAlignmentBelt(speedInput.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyorSubsystem.stopConveyorMotor();
    System.out.println("conveyor end");
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
