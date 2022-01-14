// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonCommands.autonCommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.autonCommands.DriveForwardGyroDistanceCommand;
import frc.robot.commands.autonCommands.DriveForwardDistanceCommand;
import frc.robot.commands.autonCommands.TurnToAngleCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SlalomPathAuton extends SequentialCommandGroup {

  private static final int CROSS_DISTANCE = 40;

  /** Creates a new SlalomPathAuton. */
  public SlalomPathAuton(DriveTrainSubsystem driveTrainSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(

      //LEAVE START ZONE
      driveForward(driveTrainSubsystem, 30, true),

      //INITIAL CROSS D3
      turn(driveTrainSubsystem, -45),
      shortPause(),
      driveForward(driveTrainSubsystem, CROSS_DISTANCE),

      //DRIVE TO D9
      turn(driveTrainSubsystem, 45),
      shortPause(),
      driveForward(driveTrainSubsystem, 140),

      //INITIAL CROSS D9
      turn(driveTrainSubsystem, 45),
      shortPause(),
      driveForward(driveTrainSubsystem, CROSS_DISTANCE),

      //LOOP AROUND D10
      turn(driveTrainSubsystem, -45),
      shortPause(),
      driveForward(driveTrainSubsystem, 55),
      turn(driveTrainSubsystem, -90),
      shortPause(),
      driveForward(driveTrainSubsystem, CROSS_DISTANCE),
      turn(driveTrainSubsystem, -90),
      shortPause(),
      driveForward(driveTrainSubsystem, 55),

      //CROSS BACK OVER D9
      turn(driveTrainSubsystem, -45),
      shortPause(),
      driveForward(driveTrainSubsystem, CROSS_DISTANCE),

      //RETURN TO D3
      turn(driveTrainSubsystem, 45),
      shortPause(),
      driveForward(driveTrainSubsystem, 140),

      //CROSS BACK OVER D3
      turn(driveTrainSubsystem, 45),
      shortPause(),
      driveForward(driveTrainSubsystem, CROSS_DISTANCE),

      //GO TO FINISH ZONE
      turn(driveTrainSubsystem, -45),
      shortPause(),
      driveForward(driveTrainSubsystem, 30)

    );
  }

  /**
   * Drive forward command with Gyro.
   * @param driveTrainSubsystem - The DriveTrain Subsystem instance.
   * @param distance - The distance (in inches) to drive forward.
   * @param resetGyro - Whether or not to reset the Gyro.
   * @return DriveForwardGyroDistanceCommand
   */
  public static DriveForwardGyroDistanceCommand driveForward(DriveTrainSubsystem driveTrainSubsystem, double distance, boolean resetGyro) {
    return new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.ENCODER_TICKS_PER_INCH * distance, -0.75, 0, resetGyro);
  }

  /**
   * Drive forward command without Gyro.
   * @param driveTrainSubsystem - The DriveTrain Subsystem instance.
   * @param distance - The distance (in inches) to drive forward.
   * @return DriveForwardDistanceCommand
   */
  public static DriveForwardDistanceCommand driveForward(DriveTrainSubsystem driveTrainSubsystem, double distance) {
    return new DriveForwardDistanceCommand(driveTrainSubsystem, Constants.ENCODER_TICKS_PER_INCH * distance, -0.75);
  }

  public static DriveForwardGyroDistanceCommand driveBackward(DriveTrainSubsystem driveTrainSubsystem, double distance) {
    return new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.ENCODER_TICKS_PER_INCH * distance, 0.75, 0, false);
  }

  public static TurnToAngleCommand turn(DriveTrainSubsystem driveTrainSubsystem, double angle) {
    return new TurnToAngleCommand(driveTrainSubsystem, 0.35, angle, false);
  }

  public static WaitCommand shortPause() {
    return new WaitCommand(0.25);
  }
}
