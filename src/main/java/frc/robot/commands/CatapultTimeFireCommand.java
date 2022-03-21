// auton
package frc.robot.commands;

import frc.robot.subsystems.CatapultSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class CatapultTimeFireCommand extends SequentialCommandGroup{

    // In this class, the robot will intake and shoot 2 balls, without using the ball detection grip file or AI.
    // It is only using the gyro sensor.
    // The robot will only need to go straight (no turning).

    public CatapultTimeFireCommand(CatapultSubsystem catapultSubsystem) { 
        super(
            new CatapultRunCommand(catapultSubsystem, 1),
            new WaitCommand(1.5), 
            new CatapultRunCommand(catapultSubsystem, 0.3)
        );  
    }

    
}

