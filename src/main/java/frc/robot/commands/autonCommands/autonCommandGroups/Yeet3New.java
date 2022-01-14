/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonCommands.autonCommandGroups;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.BlasterConstantOutputCommand;
import frc.robot.commands.BlasterDistanceBasedCommand;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.IndexToBlasterCommand;
import frc.robot.commands.ManualTurretCommand;
import frc.robot.commands.TurnToTargetCommand;
import frc.robot.commands.autonCommands.DriveForwardDistanceCommand;
import frc.robot.commands.autonCommands.DriveForwardGyroDistanceCommand;
import frc.robot.commands.autonCommands.TimedCommand;
import frc.robot.sensors.Lidar;
import frc.robot.subsystems.BlasterSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Yeet3New extends SequentialCommandGroup {
  /**
   * Creates a new Yeet3.
   */
  public Yeet3New(TurretSubsystem turretSubsystem, DriveTrainSubsystem driveTrainSubsystem, ConveyorSubsystem conveyorSubsystem, IntakeSubsystem intakeSubsystem, Lidar lidar, BlasterSubsystem blasterSubsystem, XboxController joystick) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new ParallelDeadlineGroup(
        // new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.AUTON_DRIVE_OFF_LINE_DISTANCE, Constants.AUTON_DRIVE_OFF_LINE_SPEED, 0, true),
        new DriveForwardDistanceCommand(driveTrainSubsystem, Constants.AUTON_DRIVE_OFF_LINE_DISTANCE, -Constants.AUTON_DRIVE_OFF_LINE_SPEED),
        new ManualTurretCommand(turretSubsystem, () -> 0, () -> 1),         
        new BlasterDistanceBasedCommand(blasterSubsystem, lidar, joystick)
      ),

      new ParallelDeadlineGroup(
        new WaitCommand(2),
        new ManualTurretCommand(turretSubsystem, () -> 0, () -> 1),
        new BlasterDistanceBasedCommand(blasterSubsystem, lidar, joystick)
      ),

     //new TimedCommand(TurnToTargetCommand(turretSubsystem, lidarSubsystem), 3000),
     new ParallelRaceGroup(
      new WaitCommand(2),
      new TurnToTargetCommand(turretSubsystem, lidar)
     ),
     
    //  new WaitCommand(5),

      new ParallelDeadlineGroup(
        new WaitCommand(5),
        // new BlasterConstantOutputCommand(blasterSubsystem, lidarSubsystem, velocityInEncoderTicks),
        new BlasterDistanceBasedCommand(blasterSubsystem, lidar, joystick),
        new IndexToBlasterCommand(intakeSubsystem),
        new ConveyorCommand(conveyorSubsystem, ()->-1)
      )
      
    );
  }
}
