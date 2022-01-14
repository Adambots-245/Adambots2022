/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.Log;
import frc.robot.Constants;
import frc.robot.sensors.Lidar;

public class TurnToTargetCommand extends CommandBase {
  private TurretSubsystem turretSubsystem;
  private Lidar lidar;
  /**
   * Creates a new TurnToTargetCommand.
   */
  public TurnToTargetCommand(TurretSubsystem turretSubsystem, Lidar lidar) {
    this.turretSubsystem = turretSubsystem;
    this.lidar = lidar;
    addRequirements(turretSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turretSubsystem.enable();

    var calculatedOffset = Math.toDegrees(Math.atan(Constants.SHOOTER_OFFSET_DISTANCE/lidar.getInches()));
    turretSubsystem.setSetpoint(calculatedOffset + Constants.CAMERA_OFFSET);

    //Log.infoF("Turret Initialized: %f", calculatedOffset);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double calculatedOffset;

    calculatedOffset = Math.toDegrees(Math.atan(Constants.SHOOTER_OFFSET_DISTANCE/lidar.getInches()));

    // turretSubsystem.setAngleOffset(calculatedOffset);
    turretSubsystem.setSetpoint(calculatedOffset + Constants.CAMERA_OFFSET);
    SmartDashboard.putNumber("angleOffset", calculatedOffset);

    //Log.infoF("Execute - AngleOffset: %f", calculatedOffset);
    //turretSystem PID loop should deal with movement
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Log.info("Auto Turret System Ended");

    if (interrupted) {
      //Log.info("Turret System Interrupted");
    }

    turretSubsystem.disable();
    turretSubsystem.stopTurret();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Log.infoF("At setpoint? %b", turretSubsystem.atSetpoint());
    return turretSubsystem.atSetpoint();
  }
}
