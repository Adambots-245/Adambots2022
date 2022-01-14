// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class PIDTuner extends PIDCommand {

  private DriveTrainSubsystem driveTrain;

  private static final double kP = 0; // Constants.GYRO_kP;
  private static final double kI = 0;
  private static final double kD = 0; // 0.01221; // Constants.GYRO_kD;

  private static double startTime = 0;
  private static double endTime = 0;

  private static double feed = 0;

  /** Creates a new TurnCommand. */
  public PIDTuner(double targetAngleDegrees, DriveTrainSubsystem drive) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        drive::getHeading,
        // This should return the setpoint (can also be a constant)
        targetAngleDegrees,
        // This uses the output
        output -> {

          // use pidtuner.com. The output of the controller is the input for Pidtuner.com
          drive.arcadeDrive(0, feed);

          endTime = System.currentTimeMillis();

          double elapsed = (endTime - startTime) / 1000; // convert to seconds
          System.out.printf(">>%f,%f,%f\n", elapsed, feed, drive.getHeading());

          feed = feed + 0.01;
        }, drive);

    this.driveTrain = drive;

    frc.robot.sensors.Gyro.getInstance().reset();

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is
    // stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(Constants.GYRO_TOLERANCE); // , Constants.GYRO_RATE_TOLERANCE_DEG_PER_SEC);
  }

  @Override
  public void initialize() {
    super.initialize();

    startTime = System.currentTimeMillis();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    frc.robot.sensors.Gyro.getInstance().reset();
  }
}
