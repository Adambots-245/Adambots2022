package frc.robot.commands.autonCommands;

import frc.robot.commands.CatapultFireCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CatapultSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;

public class Blind1 extends SequentialCommandGroup{
    public Blind1(CatapultSubsystem shoot, DriveTrainSubsystem drivetrain){
    super(
        //wait
        new WaitCommand(1), 
        //shoot
        new CatapultFireCommand(shoot),
        //drive back
        new DriveForwardDistanceCommand(drivetrain, 4, 0.5)
        );
    }
}
