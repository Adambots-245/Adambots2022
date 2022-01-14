/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TurnToAngleNoPIDCommand extends CommandBase {
  /**
   * Creates a new DriveForwardDistance.
   */
  DriveTrainSubsystem driveTrain;
  private Gyro gyro;
  private double targetAngle;
  private boolean resetGyro = true;
  private double inputSpeed = 0.8; //don't turn too fast

  public TurnToAngleNoPIDCommand(DriveTrainSubsystem inpuDriveTrain, double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = inpuDriveTrain;

    this.targetAngle = targetAngle;

    // gyroPIDSubsystem = new GyroPIDSubsystem();
    gyro = Gyro.getInstance();
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (resetGyro) {
      gyro.reset();
    }

    this.targetAngle += gyro.getYaw(); //if not set to zero, adjust to whatever the initial value is

    driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnSpeed = inputSpeed * Math.signum(targetAngle); 
    System.out.println("executing turn to angle");
    System.out.println("yaw:" + gyro.getYaw());
    SmartDashboard.putNumber("yaw", gyro.getYaw());
    // SmartDashboard.putNumber("yaw",gyroPIDSubsystem.getGyroSubsystem().getYaw());
    // SmartDashboard.putNumber("gyroPIDSubsystem.getMeasurement()", gyroPIDSubsystem.getMeasurement());

    SmartDashboard.putNumber("turnSpeed", turnSpeed);
    SmartDashboard.putNumber("leftSpeed", driveTrain.getLeftDriveEncoderVelocity());
    SmartDashboard.putNumber("rightSpeed", driveTrain.getRightDriveEncoderVelocity());

    driveTrain.arcadeDrive(0, turnSpeed);
    // driveTrain.driveDistance(distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("turn to angle interrupted");
    }

    driveTrain.arcadeDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;

    // return gyroPIDSubsystem.getController().atSetpoint();
    if (targetAngle < 0)
      return gyro.getYaw() <= targetAngle;
    else
      return gyro.getYaw() >= targetAngle;
  }
}
