/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GyroPIDSubsystem;

public class TurnToAngleCommand extends CommandBase {
  /**
   * Creates a new DriveForwardDistance.
   */
  DriveTrainSubsystem driveTrain;
  double speed;
  private GyroPIDSubsystem gyroPIDSubsystem;
  private Gyro gyro;
  private double targetAngle;
  private boolean resetGyro = true;
  private double pidValue;
  private double accumulate = 0;


  public TurnToAngleCommand(DriveTrainSubsystem inpuDriveTrain, double targetAngle, boolean resetGyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = inpuDriveTrain;
    // speed = inputSpeed;
    speed = 0;

    this.targetAngle = targetAngle;

    gyroPIDSubsystem = new GyroPIDSubsystem();
    gyro = Gyro.getInstance();

    gyroPIDSubsystem.getController().enableContinuousInput(-180, 180);

    addRequirements(driveTrain);
  }

  // TurnToAngleCommand(DriveTrainSubsystem inpuDriveTrain, double inputSpeed,
  //     double targetAngle) {
  //   this(inpuDriveTrain, targetAngle, true);
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    accumulate = 0;

    if (resetGyro) {
      gyro.reset();
    }

    driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pidValue = gyroPIDSubsystem.getController().calculate(gyroPIDSubsystem.getMeasurement(), targetAngle);
    if (gyroPIDSubsystem.getController().atSetpoint()) {
      accumulate++;
    }
    else {
      accumulate = 0;
    }

    double turnSpeed = pidValue;
    if (pidValue < 0) turnSpeed -= Constants.FEED_FORWARD;
    if (pidValue > 0) turnSpeed += Constants.FEED_FORWARD;
    // double upperLimit = 0.45;
    // double lowerLimit = 0.03;
    // if (pidValue < upperLimit && pidValue > lowerLimit){
    //   turnSpeed = upperLimit;
    // }
    // if (pidValue > -upperLimit && pidValue < -lowerLimit){
    //   turnSpeed = -upperLimit;
    // }
    // if (pidValue < lowerLimit && pidValue > -lowerLimit){
    //   turnSpeed = 0;
    // }
    // System.out.println("Turnspeed: " + turnSpeed);
    
    // System.out.println("executing turn to angle");
    // System.out.println("yaw:" + gyro.getYaw());
    SmartDashboard.putNumber("yaw", gyro.getYaw());
    // SmartDashboard.putNumber("yaw",gyroPIDSubsystem.getGyroSubsystem().getYaw());
    // SmartDashboard.putNumber("gyroPIDSubsystem.getMeasurement()", gyroPIDSubsystem.getMeasurement());

    // SmartDashboard.putNumber("leftSpeed", driveTrain.getLeftDriveEncoderVelocity());
    // SmartDashboard.putNumber("rightSpeed", driveTrain.getRightDriveEncoderVelocity());

    // driveTrain.arcadeDrive(speed, turnSpeed/Math.abs(turnSpeed));
    
    // if (turnSpeed > 0) turnSpeed = Math.max(turnSpeed, 0.35);
    // else turnSpeed = Math.min(turnSpeed, -0.35);

    // turnSpeed = Math.max(turnSpeed, 0.35 * Integer.signum((int)turnSpeed));
    // turnSpeed = MathUtil.clamp(turnSpeed, 0.35, 1)
    driveTrain.arcadeDrive(speed, turnSpeed, false);
    // driveTrain.driveDistance(distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("turn to angle interrupted");
    }
    // gyro.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

    // return accumulate >= 10;
  }
}
