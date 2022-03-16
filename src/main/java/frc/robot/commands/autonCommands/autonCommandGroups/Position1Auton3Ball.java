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

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;


public class Position1Auton3Ball extends SequentialCommandGroup{

    // In this class, the robot will intake and shoot 2 balls, without using the ball detection grip file or AI.
    // It is only using the gyro sensor.
    // The robot will only need to go straight (no turning).

    public Position1Auton3Ball(DriveTrainSubsystem driveTrain, IntakeSubsystem intakeSubsystem, CatapultSubsystem catapultSubsystem) { 
        super(
            new CatapultFireCommand(catapultSubsystem), //shoot first ball
     
            new WaitCommand(0.5),
            new AutonStartIntakeCommand(intakeSubsystem, () -> -1),
            new DriveForwardDistanceCommand(driveTrain, Constants.ENCODER_TICKS_PER_INCH * 42, -0.75), //suck the 2nd ball
            new WaitCommand(4), // wait for intake to suck ball
            new CatapultFireCommand(catapultSubsystem), //fire 2nd ball

            new TurnToAngleCommand(driveTrain, 0.3, -58, true),
            new DriveForwardDistanceCommand(driveTrain, Constants.ENCODER_TICKS_PER_INCH * 110, -1), //suck 3rd ball
            new TurnToHubCommand(driveTrain, 1), //allign to hub
            new AllignToHubCommand(driveTrain),
            new WaitCommand(4), // wait for intake to suck ball
            new CatapultFireCommand(catapultSubsystem), //shoot 3rd ball
            new StopIntakeOuttakeCommand(intakeSubsystem)
           

        );  
    }
}

