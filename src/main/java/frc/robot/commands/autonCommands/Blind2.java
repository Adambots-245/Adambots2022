package frc.robot.commands.autonCommands;
import frc.robot.commands.StartIntakeCommand;
import frc.robot.commands.StopIntakeOuttakeCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.DoubleSupplier;
import javax.sound.midi.Sequence;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


//@SuppressWarnings

public class Blind2 extends SequentialCommandGroup{
    public Blind2(DriveTrainSubsystem drivetrain, IntakeSubsystem intake){
    super(
        new WaitCommand(1), 
        //shoot
        new StartIntakeCommand(intake, () -> 1),
        new DriveForwardDistanceCommand(drivetrain, 4, 0.5),
        new WaitCommand(2),
        new StopIntakeOuttakeCommand(intake),
        new WaitCommand(2), 
        new DriveForwardDistanceCommand(drivetrain, 4, -0.5)
        );
        //shoot

    }
}

