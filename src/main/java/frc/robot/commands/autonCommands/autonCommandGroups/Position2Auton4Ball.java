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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.commands.autonCommands.*;
import frc.robot.sensors.Lidar;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.DriverStation;


public class Position2Auton4Ball extends SequentialCommandGroup{

    // In this class, the robot will intake and shoot 2 balls, without using the ball detection grip file or AI.
    // It is only using the gyro sensor.
    // The robot will only need to go straight (no turning).

    public Position2Auton4Ball(DriveTrainSubsystem driveTrain, IntakeSubsystem intake, CatapultSubsystem catapultSubsystem) { 
        super(
            // shoot first ball
            new CatapultFireCommand(catapultSubsystem),
            // suck 2nd ball
            new ParallelCommandGroup(
                new StartIntakeCommand(intake, () -> -1),
                new DriveForwardDistanceCommand(driveTrain, Constants.ENCODER_TICKS_PER_INCH * 42, 0.75)
            ),
            new WaitCommand(3),
            // shoot 2nd ball
            new CatapultFireCommand(catapultSubsystem),
            //suck 3rd and 4th ball
            new TurnToAngleCommand(driveTrain, 0.5, 25 * (DriverStation.getAlliance() == Alliance.Red?1:-1), true),
            new DriveForwardDistanceCommand(driveTrain, 120, 1),
            //allign to hub
            new TurnToAngleCommand(driveTrain, 0.5, -20 * (DriverStation.getAlliance() == Alliance.Red?1:-1), true),
            new CatapultFireCommand(catapultSubsystem),
            new StopIntakeOuttakeCommand(intake)
        );

    
    }
}

