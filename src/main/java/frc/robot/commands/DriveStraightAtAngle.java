/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveStraightAtAngle extends PIDCommand {
  /**
   * Creates a new DriveStraight.
   */
  private DriveTrainSubsystem driveTrain;
  private double targetDistance;
  private double targetAngle;
  public DriveStraightAtAngle(DriveTrainSubsystem driveTrain, double targetAngle, double forwardSpeed, double targetDistance) {
    super(
        // The controller that the command will use
        new PIDController(Constants.GYRO_kP, Constants.GYRO_kI, Constants.GYRO_kD),
        // This should return the measurement
        () -> driveTrain.getHeading(),
        // This should return the setpoint (can also be a constant)
        () -> targetAngle,
        // This uses the output
        output -> { 
          // Use the output here
          driveTrain.arcadeDrive(forwardSpeed, output);

          System.out.println("Driving Straight: " + driveTrain.getHeading());
        },
        driveTrain);

        this.driveTrain = driveTrain;
        this.targetDistance = targetDistance;
        this.targetAngle = targetAngle;

        // addRequirements(driveTrainSubsystem);
        // driveTrainSubsystem.resetGyro(true);

    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }
  @Override
  public void initialize() {
    super.initialize();
    // driveTrainSubsystem.resetEncoders();
    // driveTrainSubsystem.resetGyro(true);

    driveTrain.resetEncoders();

    System.out.println("Initialize - Heading:"+ driveTrain.getHeading());
    double relativeSetPoint = driveTrain.getHeading() + this.targetAngle; 
    
    // relativeSetPoint = Math.signum(this.targetAngle) * relativeSetPoint;
    getController().setSetpoint(relativeSetPoint);
    System.out.println("Initialize - SetPoint:"+ relativeSetPoint);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (driveTrain.getAverageDriveEncoderValue() >= targetDistance);    
  }
}
