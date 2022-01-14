/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonCommands.autonCommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.autonCommands.DriveForwardGyroDistanceCommand;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class CrossBaseline extends SequentialCommandGroup {
  /**
   * Creates a new CrossBaseline.
   */
  public CrossBaseline(DriveTrainSubsystem driveTrainSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.AUTON_DRIVE_OFF_LINE_DISTANCE, -Constants.AUTON_DRIVE_OFF_LINE_SPEED, 0, true));
  }
}
