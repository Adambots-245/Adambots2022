// auton
package frc.robot.commands.autonCommands.autonCommandGroups;

import frc.robot.Constants;
import frc.robot.subsystems.CatapultSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.HangSubsystem;
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

    public Auton2Ball(DriveTrainSubsystem driveTrain, IntakeSubsystem intakeSubsystem, CatapultSubsystem catapultSubsystem, HangSubsystem hangSubsystem) { 
        super(
            new IntakeOutCommand(intakeSubsystem),
            new MoveHangOutCommand(hangSubsystem),

            new CatapultTimeFireCommand(catapultSubsystem), //shoot first ball
            new AutonStartIntakeCommand(intakeSubsystem, () -> -1),
   
            // new DriveForwardDistanceCommand(driveTrain, Constants.ENCODER_TICKS_PER_INCH * Utils.firstDistance(Utils.BallPosition.ONE), -0.75)
            new ParallelCommandGroup(
                new DriveForwardDistanceCommand(driveTrain, Constants.ENCODER_TICKS_PER_INCH * 16.5, -0.75), // Drive until intake second ball
                new BandMoveCommand(catapultSubsystem, Constants.SECOND_BALL_AUTON_TENSION) //Tension for second ball shot
            ),
    
            new AutonSmartIntakeCommand(intakeSubsystem, -1.0), //Waiting for intake to suck ball into catapult
            new WaitCommand(0.5),
            new CatapultTimeFireCommand(catapultSubsystem), // shoot second ball
            new BandMoveCommand(catapultSubsystem, Constants.TARMAC_TENSION),
            new StopIntakeCommand(intakeSubsystem)
        );  
    }

    
}

