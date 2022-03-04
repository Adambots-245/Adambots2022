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


public class Auton2Ball extends SequentialCommandGroup{

    // In this class, the robot will intake and shoot 2 balls, without using the ball detection grip file or AI.
    // It is only using the gyro sensor.
    // The robot will only need to go straight (no turning).

    public Auton2Ball(DriveTrainSubsystem driveTrain, IntakeSubsystem intakeSubsystem, CatapultSubsystem catapultSubsystem, GyroPIDSubsystem gyro) { 
        super(
            // shoot first ball
            new CatapultFireCommand(catapultSubsystem),
            //suck second ball
            new ParallelCommandGroup(
                new StartIntakeCommand(intakeSubsystem, () -> -1),
                new DriveForwardDistanceCommand(driveTrain, Constants.ENCODER_TICKS_PER_INCH * firstDistance(), 0.75)
            ),
            //shoot 2nd ball
            new ParallelCommandGroup(
                new WaitCommand(3),
                new TurnToHubCommand(driveTrain, -1),
                new AllignToHubCommand(driveTrain)
            ),
            new CatapultFireCommand(catapultSubsystem),
            new StopIntakeOuttakeCommand(intakeSubsystem)
        );  
    }

    public static int firstDistance(){
        int distance = 0;

        switch(DriverStation.getLocation()) {
            case 1:
                distance = 42;
                break;
            case 2:
                distance = 42;
                break;
            case 3:
                distance = 42;
                break;
            default:
                distance = 42;
                break;    
        }
        return distance;
    }
}

