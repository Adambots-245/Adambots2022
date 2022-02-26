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
        new WaitCommand(1), 
        new CatapultFireCommand(shoot),
        new StartIntakeCommand(intake, () -> 1),
        new DriveForwardDistanceCommand(drivetrain, 4, 0.5),
        new WaitCommand(2),
        new StopIntakeOuttakeCommand(intake),
        new WaitCommand(2), 
        new DriveForwardDistanceCommand(drivetrain, 4, -0.5),
        new CatapultFireCommand(shoot)
        );
    }
}

