// auton
package frc.robot.commands.autonCommands.autonCommandGroups;

import frc.robot.Constants;
import frc.robot.subsystems.CatapultSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.commands.autonCommands.*;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.Utils;


public class Auton2Ball extends SequentialCommandGroup{

    // In this class, the robot will intake and shoot 2 balls, without using the ball detection grip file or AI.
    // It is only using the gyro sensor.
    // The robot will only need to go straight (no turning).

    public Auton2Ball(DriveTrainSubsystem driveTrain, IntakeSubsystem intakeSubsystem, CatapultSubsystem catapultSubsystem) { 
        super(
            new CatapultFireCommand(catapultSubsystem),

            //suck second ball
            new AutonStartIntakeCommand(intakeSubsystem, () -> -1),
   
                // new DriveForwardDistanceCommand(driveTrain, Constants.ENCODER_TICKS_PER_INCH * Utils.firstDistance(Utils.BallPosition.ONE), -0.75)
            new ParallelCommandGroup(
                new DriveForwardDistanceCommand(driveTrain, Constants.ENCODER_TICKS_PER_INCH * 34, -0.75),
                new BandMoveCommand(catapultSubsystem, 3.1) //Tension for second ball shot
            ),
                
            /*
            //shoot 2nd ball
            new ParallelCommandGroup(
                new WaitCommand(2),
                new TurnToHubCommand(driveTrain, -1)
            ),
            new AllignToHubCommand(driveTrain),
            */
            new WaitCommand(4), //Waiting for intake to suck ball into catapult
            new StopIntakeOuttakeCommand(intakeSubsystem), //Stopping intake after ball is in the catapult
            new CatapultFireCommand(catapultSubsystem),
            new BandMoveCommand(catapultSubsystem, Constants.TARMAC_TENSION)
        );  
    }

    
}

