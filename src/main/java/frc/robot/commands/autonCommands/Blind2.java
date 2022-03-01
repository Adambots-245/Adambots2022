package frc.robot.commands.autonCommands;


import frc.robot.commands.CatapultFireCommand;
import frc.robot.commands.StartIntakeCommand;
import frc.robot.commands.StopIntakeOuttakeCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CatapultSubsystem;


public class Blind2 extends SequentialCommandGroup{
    public Blind2(DriveTrainSubsystem drivetrain, IntakeSubsystem intake, CatapultSubsystem shoot){
    super(
        //wait
        new WaitCommand(1), 
        //fire
        new CatapultFireCommand(shoot),
        //startintake
        new StartIntakeCommand(intake, () -> 1),
        //drive froward towards ball
        new DriveForwardDistanceCommand(drivetrain, 4, 0.5),
        // wait
        new WaitCommand(2),
        //stop intake command 
        new StopIntakeOuttakeCommand(intake),
        // wait
        new WaitCommand(2), 
        //drave forwards towards hub 
        new DriveForwardDistanceCommand(drivetrain, 4, -0.5),
        // Fire
        new CatapultFireCommand(shoot)
        );
    }
}

