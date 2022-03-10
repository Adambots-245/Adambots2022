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
import frc.robot.utils.Utils;
import edu.wpi.first.wpilibj.DriverStation;


public class Auton2BallColor extends SequentialCommandGroup{

    // In this class, the robot will intake and shoot 2 balls, without using the ball detection grip file or AI.
    // It is only using the gyro sensor.
    // The robot will only need to go straight (no turning).

    public Auton2BallColor(DriveTrainSubsystem driveTrain, IntakeSubsystem intakeSubsystem, CatapultSubsystem catapultSubsystem) { 
        super(
            // shoot first ball
            new CatapultFireCommand(catapultSubsystem),
            new WaitCommand(0.5),
            //suck second ball
            new AutonStartIntakeCommand(intakeSubsystem, () -> -1),
            // new ParallelDeadlineGroup(
            //     new WaitCommand(0.5),
            //     new StartIntakeCommand(intakeSubsystem, () -> -1)
            // ),       
                // new DriveForwardDistanceCommand(driveTrain, Constants.ENCODER_TICKS_PER_INCH * Utils.firstDistance(Utils.BallPosition.ONE), -0.75)
            new ParallelCommandGroup(
                new DriveForwardDistanceCommand(driveTrain, Constants.ENCODER_TICKS_PER_INCH * 30, -0.75),
                new BandMoveCommand(catapultSubsystem, 4.5)
            ),
                
            /*
            //shoot 2nd ball
            new ParallelCommandGroup(
                new WaitCommand(2),
                new TurnToHubCommand(driveTrain, -1)
            ),
            new AllignToHubCommand(driveTrain),
            */
            new WaitCommand(5),
            new CatapultFireCommand(catapultSubsystem),
            new StopIntakeOuttakeCommand(intakeSubsystem),
            new BandMoveCommand(catapultSubsystem, 4.4)
        );  
    }

    
}

