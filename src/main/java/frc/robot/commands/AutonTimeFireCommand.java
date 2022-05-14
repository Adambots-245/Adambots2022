// auton
package frc.robot.commands;

import frc.robot.subsystems.CatapultSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class AutonTimeFireCommand extends SequentialCommandGroup{

    // In this class, the robot will intake and shoot 2 balls, without using the ball detection grip file or AI.
    // It is only using the gyro sensor.
    // The robot will only need to go straight (no turning).

    public AutonTimeFireCommand(CatapultSubsystem catapultSubsystem) { 
        super(
            new CatapultRunCommand(catapultSubsystem, 0.5),
            new WaitCommand(0.2)
        );  
    }

    
}

