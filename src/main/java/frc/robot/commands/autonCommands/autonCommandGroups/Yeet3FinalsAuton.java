/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonCommands.autonCommandGroups;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.BlasterDistanceBasedCommand;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.IndexToBlasterCommand;
import frc.robot.commands.LowerIntakeArmCommand;
import frc.robot.commands.ManualTurretCommand;
import frc.robot.commands.StartIntakeCommand;
import frc.robot.commands.TurnToTargetCommand;
import frc.robot.commands.autonCommands.DriveForwardGyroDistanceCommand;
import frc.robot.commands.autonCommands.TimedBlasterDistanceBasedCommand;
import frc.robot.commands.autonCommands.TimedCommand;
import frc.robot.commands.autonCommands.TimedManualTurretCommand;
import frc.robot.sensors.Lidar;
import frc.robot.subsystems.BlasterSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Yeet3FinalsAuton extends SequentialCommandGroup {
  /**
   * Creates a new Yeet3.
   */
  public Yeet3FinalsAuton(TurretSubsystem turretSubsystem, DriveTrainSubsystem driveTrainSubsystem, ConveyorSubsystem conveyorSubsystem, IntakeSubsystem intakeSubsystem, Lidar lidarSubsystem, BlasterSubsystem blasterSubsystem, XboxController joystick) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      // new ParallelCommandGroup(
        // new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.AUTON_DRIVE_OFF_LINE_DISTANCE,-Constants.AUTON_DRIVE_OFF_LINE_SPEED, 0, true),//true
        new LowerIntakeArmCommand(intakeSubsystem),

        new TimedManualTurretCommand(turretSubsystem, () -> 0, () -> 1, 1100),
        new TimedManualTurretCommand(turretSubsystem, () -> 0, () -> 0, 300),
      // ),

    //  new TimedCommand(new TurnToTargetCommand(turretSubsystem, lidarSubsystem), 3000),
    // new TurnToTargetCommand(turretSubsystem, lidarSubsystem),
    new TimedManualTurretCommand(turretSubsystem, () -> 0, () -> 0, 100),
    // new TimedBlasterDistanceBasedCommand(blasterSubsystem, lidarSubsystem, 20),
    new TimedBlasterDistanceBasedCommand(blasterSubsystem, lidarSubsystem, 2000), //UNCOMMENT THIS
    new ParallelDeadlineGroup(
      new WaitCommand(10),
      new BlasterDistanceBasedCommand(blasterSubsystem, lidarSubsystem, joystick), //UNCOMMENT THIS
      new IndexToBlasterCommand(intakeSubsystem),
      new ConveyorCommand(conveyorSubsystem, ()->-.75),
      new StartIntakeCommand(intakeSubsystem, ()-> -.5)
    )
    // new DriveForwardGyroDistanceCommand(driveTrainSubsystem, 28*3500, -Constants.AUTON_DRIVE_OFF_LINE_SPEED, 45, true)
    );
  }
}
