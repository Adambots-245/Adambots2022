// auton
package frc.robot.commands.autonCommands.autonCommandGroups;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.CatapultSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GyroPIDSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;


public class BallPrimeAndFireCommandGroup extends SequentialCommandGroup{

    // In this class, the robot will intake and shoot 2 balls, without using the ball detection grip file or AI.
    // It is only using the gyro sensor.
    // The robot will only need to go straight (no turning).

    public BallPrimeAndFireCommandGroup (CatapultSubsystem catapultSubsystem) { 
        super(
          //shoot ball
          // new ParallelDeadlineGroup(
          //   new WaitCommand(2),
          //   new CatapultRunCommand(catapultSubsystem)
          // )

            new CatapultPrimeCommand(catapultSubsystem),
            // new WaitCommand(0.5), //Delay between priming and firing to ensure a complete fire and make it easier to debug
            new CatapultFireCommand(catapultSubsystem)
        );  
    }
}

