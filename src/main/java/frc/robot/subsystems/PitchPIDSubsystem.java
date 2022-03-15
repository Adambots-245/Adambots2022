/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.sensors.Gyro;

public class PitchPIDSubsystem extends PIDSubsystem {
  /**
   * Creates a new GyroPIDSubsystem.
   */
  private static double kP = 10;
  private static double kI = 0;
  private static double kD = 0;
  
  private Gyro gyro;
  private double output;

  public PitchPIDSubsystem() {
    super(new PIDController(kP, kI, kD));
  
    getController().setTolerance(Constants.GYRO_TOLERANCE);
    setSetpoint(0);
  
    gyro = Gyro.getInstance();

    enable();
  }

  public Gyro getGyro(){
    return gyro;
  }

  public double getOutput(){
    return output;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    this.output = output;
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return gyro.getPitch();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }
}
