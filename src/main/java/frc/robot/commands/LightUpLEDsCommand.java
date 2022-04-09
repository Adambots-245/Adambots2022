// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CANdleSubsystem;

public class LightUpLEDsCommand extends CommandBase {
  private CANdleSubsystem candleSubsystem;
  private NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private NetworkTable table;
  private NetworkTableEntry hubAngleEntry;

  /** Creates a new LightUpLEDs. */
  public LightUpLEDsCommand(CANdleSubsystem candleSubsystem) {
    
    this.candleSubsystem = candleSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(candleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    table = instance.getTable(Constants.VISION_TABLE_NAME);
    hubAngleEntry = table.getEntry(Constants.HUB_ANGLE_ENTRY_NAME);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lightUpAlignment();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void lightUpAlignment(){
    
    double targetAngle = Math.abs(hubAngleEntry.getDouble(Constants.ANGLE_NOT_DETECTED));
  
    // RobotMap.AlignLight.set(Value.kOff);

    System.out.println("Light Up: " + targetAngle);

    if (targetAngle != Constants.ANGLE_NOT_DETECTED){
      if (targetAngle > 0 && targetAngle <= Constants.ANGLE_RANGE){
        // RobotMap.AlignLight.set(Value.kOn);

        // candleSubsystem.setColor(Constants.LED_COLOR);
        candleSubsystem.setColor(255, 255, 0);
      }
    } else {
      // RobotMap.AlignLight.set(Value.kOff);
      candleSubsystem.setColor(0, 0, 0);
    }
  }
}
