/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class GyroDriveForDistCommand extends CommandBase {
  /**
   * Creates a new DriveForwardDistance.
   */
  DriveTrainSubsystem driveTrain;
  double distance;
  double speed;
  double yaw;

  public GyroDriveForDistCommand(DriveTrainSubsystem inpuDriveTrain, double inputDistance, double inputSpeed, float getYaw) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = inpuDriveTrain;
    distance = inputDistance;
    speed = inputSpeed;
    yaw = getYaw;

    addRequirements(driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnSpeed;
    if (yaw >= 20.0){
      turnSpeed = 0.1;
    } else if (yaw <= -20.0) {
      turnSpeed = -0.1;
    } else {
      turnSpeed = 0;
    }
    driveTrain.arcadeDrive(speed, turnSpeed);
    //driveTrain.driveDistance(distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (driveTrain.getAverageDriveEncoderValue() >= distance);
  }
}
