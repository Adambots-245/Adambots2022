// auton
package frc.robot.commands.autonCommands.autonCommandGroups;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.CatapultSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GyroPIDSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.commands.autonCommands.*;
import frc.robot.sensors.Lidar;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.DriverStation;


public class Position1Auton5Ball extends SequentialCommandGroup{

    // In this class, the robot will intake and shoot 2 balls, without using the ball detection grip file or AI.
    // It is only using the gyro sensor.
    // The robot will only need to go straight (no turning).

    public Position1Auton5Ball(DriveTrainSubsystem driveTrain, IntakeSubsystem intakeSubsystem, CatapultSubsystem catapultSubsystem) { 
        super(
            //fire first ball
            new CatapultFireCommand(catapultSubsystem),
            //suck the 2nd ball
            new ParallelCommandGroup(
                new StartIntakeCommand(intakeSubsystem, () -> -1),
                new DriveForwardDistanceCommand(driveTrain, Constants.ENCODER_TICKS_PER_INCH * 42, 0.75)
            ),
            //suck the 3rd ball
            new TurnToAngleCommand(driveTrain, 0.5, -58, true),
            new DriveForwardDistanceCommand(driveTrain, Constants.ENCODER_TICKS_PER_INCH * 110, 1),
            new ParallelRaceGroup(
                new WaitCommand(6),
                new TurnToHubCommand(driveTrain, 1)
            ),
            new AllignToHubCommand(driveTrain),
            //fire second and third ball
            new CatapultFireCommand(catapultSubsystem),
            new WaitCommand(3),
            new CatapultFireCommand(catapultSubsystem),
            //drive to fourth ball
            new TurnToAngleCommand(driveTrain, 0.5, -50, true), //Wrong angle
            new DriveForwardDistanceCommand(driveTrain, 120, 1),
            new WaitCommand(3),
            new TurnToAngleCommand(driveTrain, 0.5, 160, true),
            new DriveForwardDistanceCommand(driveTrain, 80, 1),
            //fire fourth and fifth ball
            new CatapultFireCommand(catapultSubsystem),
            new WaitCommand(3),
            new CatapultFireCommand(catapultSubsystem),
            new StopIntakeOuttakeCommand(intakeSubsystem)
        );  
    }
}

