package frc.robot.commands.autonCommands.autonCommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.BlasterDistanceBasedCommand;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.IndexToBlasterCommand;
import frc.robot.commands.LowerIntakeArmCommand;
import frc.robot.commands.autonCommands.DriveForwardDistanceCommand;
import frc.robot.commands.autonCommands.DriveForwardGyroDistanceCommand;
import frc.robot.commands.autonCommands.TimedBlasterDistanceBasedCommand;
import frc.robot.commands.autonCommands.TimedConveyorCommand;
import frc.robot.commands.autonCommands.TimedStartIntakeCommand;
import frc.robot.sensors.Lidar;
import frc.robot.subsystems.*;


public class GyroTwoPracticeTest extends SequentialCommandGroup {

    public GyroTwoPracticeTest (DriveTrainSubsystem driveTrain) {
        super(
            new DriveForwardDistanceCommand(driveTrain, Constants.ENCODER_TICKS_PER_INCH * 12, 0.5)
        );
    }

    

}
