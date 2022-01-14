// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveStraightCommand extends PIDCommand {

  private static final double kP = Constants.GYRO_kP;
  private static final double kI = Constants.GYRO_kI;
  private static final double kD = Constants.GYRO_kD;

  private final DriveTrainSubsystem driveTrain;
  private final double distance;

  /** Creates a new DriveStraightCommand. */
  public DriveStraightCommand(DriveTrainSubsystem drive, double speed, double distance) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        drive::getTurnRate,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          drive.arcadeDrive(speed, output);
        }, drive);

    this.driveTrain = drive;
    this.distance = distance;

    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    SmartDashboard.putData("Straight PID Controller", getController());
    // Set the controller input to be continuous (because it is an angle controller
    // and getYaw returns values from -180 to 180)
    getController().enableContinuousInput(-180, 180);

    // Set the controller tolerance - the delta tolerance ensures the robot is
    // stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(Constants.GYRO_TOLERANCE); // , Constants.GYRO_RATE_TOLERANCE_DEG_PER_SEC);

  }

  @Override
  public void initialize() {
    super.initialize();

    driveTrain.resetEncoders();
    driveTrain.resetGyro(true);

    System.out.println("Initialize - Heading:" + driveTrain.getHeading());
    double relativeSetPoint = driveTrain.getHeading(); // stay at whatever angle is current value

    // relativeSetPoint = Math.signum(this.targetAngle) * relativeSetPoint;
    getController().setSetpoint(relativeSetPoint);
    System.out.println("Initialize - SetPoint:" + relativeSetPoint);

    // System.out.println("Heading after reset: " + driveTrain.getHeading());
    // System.out.println("Yaw after reset: " + Gyro.getInstance().getYaw());
    // System.out.println("Angle after reset: " + Gyro.getInstance().getAngle());

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (driveTrain.getAverageDriveEncoderValue() >= distance);
  }
}
