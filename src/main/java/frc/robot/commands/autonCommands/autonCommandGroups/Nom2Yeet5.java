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
public class Nom2Yeet5 extends SequentialCommandGroup {
  /**
   * Creates a new Yeet3PushNom3.
   */
  public Nom2Yeet5(DriveTrainSubsystem driveTrainSubsystem, IntakeSubsystem intakeSubsystem,
      TurretSubsystem turretSubsystem, BlasterSubsystem blasterSubsystem, Lidar lidar,
      ConveyorSubsystem conveyorSubsystem, XboxController joystick) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
    super(
        new LowerIntakeArmCommand(intakeSubsystem),
        new ShiftLowGearCommand(driveTrainSubsystem),
        // NOM/INTAKE 2 BALLS (also keep driving (parallel to balls and guardrail))
        new ParallelDeadlineGroup( // deadline because it should move on after it has reached the position
          new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.AUTON_2_BALL_STRAIGHT_DISTANCE *.4, -.75, 0, false),
          new ManualTurretCommand(turretSubsystem, () -> 0, () -> 1),
          new StartIntakeCommand(intakeSubsystem, () -> -1.0)
            // new ConveyorCommand(conveyorSubsystem, () -> -1.0)
        ),
        new ParallelDeadlineGroup( // deadline because it should move on after it has reached the position
          new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.AUTON_2_BALL_STRAIGHT_DISTANCE*.6, -.75, 0, false),
          new ManualTurretCommand(turretSubsystem, () -> 0, () -> 1),
          new StartIntakeCommand(intakeSubsystem, () -> -1.0),
          new ConveyorCommand(conveyorSubsystem, () -> -1.0)
        ),

        // new TurnToAngleCommand(driveTrainSubsystem, -.75, targetAngle, resetGyro),

        // YEET 5 BALLS
        
        // new ParallelDeadlineGroup(
        //   new WaitCommand(2.5),
        //   new ManualTurretCommand(turretSubsystem, () -> 0, () -> 1),
        //   new BlasterDistanceBasedCommand(blasterSubsystem, lidar, joystick)
        //   // new StartIntakeCommand(intakeSubsystem, () -> 1.0)
        // ),

        new ParallelRaceGroup(
          new WaitCommand(2),
          new TurnToTargetCommand(turretSubsystem, lidar),
          new BlasterDistanceBasedCommand(blasterSubsystem, lidar, joystick)
          ),
        // new BackboardNearCommand(blasterSubsystem),
        // new TurnToTargetCommand(turretSubsystem, lidarSubsystem),

        new ParallelDeadlineGroup(
          new WaitCommand(10), 
          new BlasterDistanceBasedCommand(blasterSubsystem, lidar, joystick),
          new IndexToBlasterCommand(intakeSubsystem),
          new StartIntakeCommand(intakeSubsystem, () -> -1.0),
          new ConveyorCommand(conveyorSubsystem, () -> -1.0)
        )
        // // NOM/INTAKE 1 BALL (also keep driving (parallel to balls and guardrail))
        // new ParallelDeadlineGroup( // deadline because it should move on after it has reached the position
        //   new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.AUTON_1_BALL_STRAIGHT_DISTANCE, -.75, 0, false),
        //   new StartIntakeCommand(intakeSubsystem, () -> 1.0)
        // )
    );
  }
}