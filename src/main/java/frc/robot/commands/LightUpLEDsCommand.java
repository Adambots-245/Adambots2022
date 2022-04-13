// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.sensors.PhotoEye;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CANdleSubsystem.AnimationTypes;

public class LightUpLEDsCommand extends CommandBase {
  private CANdleSubsystem candleSubsystem;
  private PhotoEye intakeSensor;
  private PhotoEye catapultIntakeSensor;
  private NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private NetworkTable table;
  private NetworkTableEntry hubAngleEntry;
  private int defaultR = 255;
  private int defaultG = 200;
  private int defaultB = 0;

  /** Creates a new LightUpLEDs. */
  public LightUpLEDsCommand(CANdleSubsystem candleSubsystem, PhotoEye intakeSensor, PhotoEye catapultIntakeSensor) {

    this.candleSubsystem = candleSubsystem;
    this.intakeSensor = intakeSensor;
    this.catapultIntakeSensor = catapultIntakeSensor;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(candleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    table = instance.getTable(Constants.VISION_TABLE_NAME);
    hubAngleEntry = table.getEntry(Constants.HUB_ANGLE_ENTRY_NAME);
    candleSubsystem.clearAllAnims();
    // candleSubsystem.setAnimationSpeed(0.5);
    // candleSubsystem.changeAnimation(AnimationTypes.Strobe);
    candleSubsystem.setColor(defaultR, defaultG, defaultB);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lightUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    candleSubsystem.setColor(0, 0, 0); // Set Yellow as default
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void lightUp() {
    double targetAngle = Math.abs(hubAngleEntry.getDouble(Constants.ANGLE_NOT_DETECTED));

    // RobotMap.AlignLight.set(Value.kOff);

    int r = defaultR;
    int g = defaultG;
    int b = defaultB;
    boolean flash  = false;

    System.out.println("Light Up: " + targetAngle);

    if (!catapultIntakeSensor.isDetecting() || !intakeSensor.isDetecting()) {
      r = 0;
      g = 0;
      b = 255;
      flash = true;
    }

    if (!intakeSensor.isDetecting() && !catapultIntakeSensor.isDetecting()) {
      r = 0;
      g = 255;
      b = 0;
      flash = false;
    }

    if (targetAngle <= Constants.ANGLE_RANGE && targetAngle != Constants.ANGLE_NOT_DETECTED) {
      // RobotMap.AlignLight.set(Value.kOn);
      // candleSubsystem.setColor(Constants.LED_COLOR);

      r = 255;
      g = 0;
      b = 0;
    }
    // else {
    // // RobotMap.AlignLight.set(Value.kOff);
    // candleSubsystem.setColor(0, 0, 0);
    // }
    candleSubsystem.setColor(r, g, b); // Set Yellow as default
    // if (flash) candleSubsystem.changeAnimation(AnimationTypes.Twinkle);
  }
}
