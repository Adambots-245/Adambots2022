// auton
package frc.robot.commands.autonCommands.autonCommandGroups;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GyroPIDSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.commands.autonCommands.*;
import frc.robot.sensors.Lidar;
import frc.robot.subsystems.BlasterSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class GyroShootTwoBalls extends SequentialCommandGroup{

    // In this class, the robot will intake and shoot 2 balls, without using the ball detection grip file or AI.
    // It is only using the gyro sensor.
    // The robot will only need to go straight (no turning).

    public GyroShootTwoBalls(DriveTrainSubsystem driveTrain, IntakeSubsystem intake, GyroPIDSubsystem gyro) { 
        super(
            new WaitCommand(1),
            new TurnToAngleCommand(driveTrain, 0.25, 0, true),
            new StartIntakeCommand(intake, () -> -1),
            new DriveForwardDistanceCommand(driveTrain, 2, 0.5),
            // shoot
            new DriveForwardDistanceCommand(driveTrain, 3, 0.5),
            new StopIntakeOuttakeCommand(intake);
            // shoot

        );

    
    }
}

/* /*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*

package frc.robot.commands.autonCommands.autonCommandGroups;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
public class PushNom2Yeet5Nom1 extends SequentialCommandGroup {
  /**
   * Creates a new Yeet3PushNom3.
   */

   /*
  public PushNom2Yeet5Nom1(DriveTrainSubsystem driveTrainSubsystem, IntakeSubsystem intakeSubsystem,
      TurretSubsystem turretSubsystem, BlasterSubsystem blasterSubsystem, Lidar lidar,
      ConveyorSubsystem conveyorSubsystem, XboxController joystick) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
    super(

        // PUSH OTHER ROBOT OFF LINE
        new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.AUTON_PUSH_ROBOT_DISTANCE,
            Constants.AUTON_PUSH_ROBOT_SPEED, 0, false),

        // NOM/INTAKE 2 BALLS (also keep driving (parallel to balls and guardrail))
        new ParallelDeadlineGroup( // deadline because it should move on after it has reached the position
            new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.AUTON_2_BALL_STRAIGHT_DISTANCE, -.75, 0, false),
            new StartIntakeCommand(intakeSubsystem, () -> 1.0)),

        // YEET 5 BALLS
        // new BackboardNearCommand(blasterSubsystem),
        new TurnToTargetCommand(turretSubsystem, lidar),
        new ParallelCommandGroup(
          // new WaitCommand(5), 
          new TimedCommand(new BlasterDistanceBasedCommand(blasterSubsystem, lidar, joystick), 5000/1000),
          new TimedCommand(new IndexToBlasterCommand(intakeSubsystem), 5000/1000),
          new TimedCommand(new ConveyorCommand(conveyorSubsystem, () -> 1.0), 5000/1000)
        ),
        // NOM/INTAKE 1 BALL (also keep driving (parallel to balls and guardrail))
        new ParallelDeadlineGroup( // deadline because it should move on after it has reached the position
          new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.AUTON_1_BALL_STRAIGHT_DISTANCE, -.75, 0, false),
          new StartIntakeCommand(intakeSubsystem, () -> 1.0)
        )
    );
  }
}
*/


