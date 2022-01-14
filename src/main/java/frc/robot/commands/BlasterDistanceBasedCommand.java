/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Lidar;
import frc.robot.subsystems.BlasterSubsystem;
import frc.robot.utils.Log;

public class BlasterDistanceBasedCommand extends CommandBase {
  /**
   * Creates a new BlasterDistanceBasedCommand.
   */
  BlasterSubsystem blasterSubsystem;
  private Lidar lidar;
  private double initialDistance = 0;
  private XboxController joystick;

  public BlasterDistanceBasedCommand(BlasterSubsystem blasterSubsystem, Lidar lidar, XboxController joystick) {
    this.lidar = lidar;
    this.blasterSubsystem = blasterSubsystem;
    this.joystick = joystick;

    SmartDashboard.putNumber("Blaster Velocity", blasterSubsystem.getVelocity());
    SmartDashboard.putNumber("Distance To Target", lidar.getInches());

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(blasterSubsystem);
    // //Log.infoF("Initializing Blaster Distance Command: %d", blasterSubsystem.getVelocity());

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("fps", 59);

    blasterSubsystem.setVelocity(10343);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    initialDistance = lidar.getInches();

    double distanceInFeet =  initialDistance / 12;
    // double distanceInFeet = 182/12;

    // double feetsPerSec = (-0.0047360 * Math.pow(distanceInFeet, 3)) + (0.3441226 * Math.pow(distanceInFeet, 2))
      // - (8.135303 * distanceInFeet) + 93.69801;
    double feetsPerSec = (0.02142857 * Math.pow(distanceInFeet, 2)) + (0.732857 * distanceInFeet) + 58.60;
    // feetsPerSec = feetsPerSec * 1.75;

    // feetsPerSec = SmartDashboard.getNumber("fps", 0);

    double inchesInFeet = 12;
    double secondsInMinute = 60;
    double numTicksPer100ms = 2048;
    double flyWheelDiameterInInches = 8; // inches

    double adjustFor100ms = 600;

    double targetVelocity = (inchesInFeet * secondsInMinute * feetsPerSec * numTicksPer100ms)
        / (flyWheelDiameterInInches * adjustFor100ms * Math.PI);

    blasterSubsystem.setVelocity(targetVelocity);
    SmartDashboard.putNumber("Blaster Velocity", blasterSubsystem.getVelocity());
    double rpm = (blasterSubsystem.getVelocity() * adjustFor100ms) / numTicksPer100ms;
    double vfps = (rpm/secondsInMinute) * Math.PI * (flyWheelDiameterInInches/inchesInFeet);
    SmartDashboard.putNumber("Blaster Velocity (RPM)", rpm);
    SmartDashboard.putNumber("Blaster Velocity (Feets Per Sec)", vfps);
    SmartDashboard.putNumber("Distance To Target", lidar.getInches());
    SmartDashboard.putBoolean("BLASTER ENABLED", true);
    // at velocity checker
    boolean atVelocity = false;
    double velocityThresh = 100; // ticks per 100ms
    if(targetVelocity-velocityThresh <= blasterSubsystem.getVelocity() && blasterSubsystem.getVelocity() <= targetVelocity+velocityThresh){
      atVelocity = true;
      joystick.setRumble(RumbleType.kRightRumble, 1);
    }
    SmartDashboard.putBoolean("atVelocity?", atVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("blaster distance end");
    SmartDashboard.putBoolean("BLASTER ENABLED", false);
    joystick.setRumble(RumbleType.kRightRumble, 0);
    joystick.setRumble(RumbleType.kLeftRumble, 0);
    blasterSubsystem.output(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
