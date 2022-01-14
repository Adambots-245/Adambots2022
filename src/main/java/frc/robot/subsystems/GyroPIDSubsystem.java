/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.sensors.Gyro;
import frc.robot.utils.Log;

public class GyroPIDSubsystem extends PIDSubsystem {
  /**
   * Creates a new GyroPIDSubsystem.
   */
  static double kP = Constants.GYRO_kP;
  static double kI = Constants.GYRO_kI;
  static double kD = Constants.GYRO_kD;
  
  Gyro gyro;

  public GyroPIDSubsystem() {
    super(new PIDController(kP, kI, kD));
  
    getController().setTolerance(Constants.GYRO_TOLERANCE);
    setSetpoint(0);
  
    gyro = Gyro.getInstance();

    //Log.infoF("Initializing GyroPIDSubsystem - kP=%f, kI=%f, kD=%f", kP, kI, kD);
  }

  @Override
  public void useOutput(double output, double targetAngle) {
    //setSetpoint(targetAngle);

    // Use the output here
  }

  public Gyro getGyro(){
    return gyro;
  }

  @Override
  public double getMeasurement() {
   
    // Return the process variable measurement here
    return gyro.getYaw();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }
}
