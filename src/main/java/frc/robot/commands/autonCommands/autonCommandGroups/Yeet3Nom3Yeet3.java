/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonCommands.autonCommandGroups;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.commands.autonCommands.*;
import frc.robot.sensors.Lidar;
import frc.robot.subsystems.BlasterSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Yeet3Nom3Yeet3 extends SequentialCommandGroup {
  /**
   * Creates a new Yeet3PushNom3.
   */
  public Yeet3Nom3Yeet3(DriveTrainSubsystem driveTrainSubsystem, IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem, BlasterSubsystem blasterSubsystem, Lidar lidar, ConveyorSubsystem conveyorSubsystem, XboxController joystick) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
    super(
      new LowerIntakeArmCommand(intakeSubsystem),
      new ShiftLowGearCommand(driveTrainSubsystem),
      new SetNormalSpeedCommand(driveTrainSubsystem),

      // DRIVE FORWARD AND STOP BEFORE BALL 1
      new ParallelDeadlineGroup(
        new DriveForwardGyroDistanceCommand(driveTrainSubsystem, 80*Constants.ENCODER_TICKS_PER_INCH, -0.75, 0, true), 
        new ManualTurretCommand(turretSubsystem, () -> 0, () -> 1),         
        new BlasterDistanceBasedCommand(blasterSubsystem, lidar, joystick),
        new StartIntakeCommand(intakeSubsystem, ()->-1.0)

      ), 

      // YEET 3 BALLS
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new ManualTurretCommand(turretSubsystem, () -> 0, () -> 1),
        new BlasterDistanceBasedCommand(blasterSubsystem, lidar, joystick),
        new StartIntakeCommand(intakeSubsystem, ()->-1.0)

      ),

     new ParallelRaceGroup(
      new WaitCommand(2),
      new BlasterDistanceBasedCommand(blasterSubsystem, lidar, joystick),
      new TurnToTargetCommand(turretSubsystem, lidar)
     ),

      new ParallelDeadlineGroup(
        new WaitCommand(4),
        new BlasterDistanceBasedCommand(blasterSubsystem, lidar, joystick),
        new IndexToBlasterCommand(intakeSubsystem),
        new ConveyorCommand(conveyorSubsystem, ()->-1)
      ),

      // NOM 3 BALLS
      new ParallelDeadlineGroup( // deadline because it should move on after it has reached the position
        new DriveForwardGyroDistanceCommand(driveTrainSubsystem, 75*Constants.ENCODER_TICKS_PER_INCH, -.75, 0, true), 
        new BlasterDistanceBasedCommand(blasterSubsystem, lidar, joystick),
        new ConveyorCommand(conveyorSubsystem, ()->-1),
        new StartIntakeCommand(intakeSubsystem, ()->-1.0)
        // new ManualTurretCommand(turretSubsystem, () -> 0, () -> 1)
      ),

      // // YEET 3 BALLS
      // new ParallelDeadlineGroup(
      //   new WaitCommand(2),
      //   // new ManualTurretCommand(turretSubsystem, () -> 0, () -> 1),
      //   new BlasterDistanceBasedCommand(blasterSubsystem, lidar, joystick)
      // ),
      new ParallelRaceGroup(
        new WaitCommand(2),
        new BlasterDistanceBasedCommand(blasterSubsystem, lidar, joystick),
        new TurnToTargetCommand(turretSubsystem, lidar)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(5),
        // new BlasterConstantOutputCommand(blasterSubsystem, lidarSubsystem, velocityInEncoderTicks),
        new BlasterDistanceBasedCommand(blasterSubsystem, lidar, joystick),
        new IndexToBlasterCommand(intakeSubsystem),
        new ConveyorCommand(conveyorSubsystem, ()->-1),
        new StartIntakeCommand(intakeSubsystem, ()->-1.0)
      )

      );
  }
}
