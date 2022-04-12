// auton
package frc.robot.commands.autonCommands.autonCommandGroups;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.CatapultSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GyroPIDSubsystem;
import frc.robot.subsystems.HangSubsystem;
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

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;


public class Auton3Ball extends SequentialCommandGroup{

    // In this class, the robot will intake and shoot 2 balls, without using the ball detection grip file or AI.
    // It is only using the gyro sensor.
    // The robot will only need to go straight (no turning).

    public Auton3Ball(DriveTrainSubsystem driveTrain, IntakeSubsystem intakeSubsystem, CatapultSubsystem catapultSubsystem, HangSubsystem hangSubsystem) { 
        super(
            new IntakeOutCommand(intakeSubsystem),
            new MoveHangOutCommand(hangSubsystem),

            new ParallelDeadlineGroup(
                new CatapultTimeFireCommand(catapultSubsystem), //shoot first ball
                new WaitCommand(0.1)
            ),
            new AutonStartIntakeCommand(intakeSubsystem, () -> -1),
   
            // new DriveForwardDistanceCommand(driveTrain, Constants.ENCODER_TICKS_PER_INCH * Utils.firstDistance(Utils.BallPosition.ONE), -0.75)
            new ParallelCommandGroup(
                new DriveForwardDistanceCommand(driveTrain, Constants.ENCODER_TICKS_PER_INCH * 16.5, -0.75), // Drive until intake second ball
                new BandMoveCommand(catapultSubsystem, Constants.SECOND_BALL_AUTON_TENSION) //Tension for second ball shot
            ),
    
            new ParallelDeadlineGroup(
                new AutonSmartIntakeCommand(intakeSubsystem, -1.0), //Waiting for intake to suck ball into catapult TODO: Add dealine group
                new WaitCommand(5)
            ),
            new WaitCommand(0.5),
            new ParallelDeadlineGroup(
                new CatapultTimeFireCommand(catapultSubsystem), //shoot first ball
                new WaitCommand(0.1)
            ),

            new ParallelRaceGroup(
                new TurnToAngleCommand(driveTrain, 119, true),
                new WaitCommand(1.25)
            ),
            new WaitCommand(0.3),
            new DriveForwardDistanceCommand(driveTrain, Constants.ENCODER_TICKS_PER_INCH * 50, -0.75), //suck 3rd ball
            new WaitCommand(0.5),
            // new TurnToHubCommand(driveTrain, 1), //allign to hub
            // new AllignToHubCommand(driveTrain),
            
            new ParallelRaceGroup(
                new TurnToAngleCommand(driveTrain, -62, true),
                new WaitCommand(1.25)
            ),

            new AutonSmartIntakeCommand(intakeSubsystem, -1.0), //Waiting for intake to suck ball into catapult
            new WaitCommand(0.5),
            new StopIntakeCommand(intakeSubsystem),
            new ParallelDeadlineGroup(                          //shoot 3rd ball
                new CatapultTimeFireCommand(catapultSubsystem), //shoot first ball
                new WaitCommand(0.1)
            ),
            new BandMoveCommand(catapultSubsystem, Constants.TARMAC_TENSION)
        );  
    }
}

