/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class PitchPIDSubsystem extends PIDSubsystem {
  /**
   * Creates a new GyroPIDSubsystem.
   */
  private Accelerometer accelerometer;

  private static double kP = 10;
  private static double kI = 0;
  private static double kD = 0;

  private double maxXAccel = 0;
  private double maxYAccel = 0;
  private double maxZAccel = 0;
  
  private double output;

  public PitchPIDSubsystem(Accelerometer accelerometer) {
    super(new PIDController(kP, kI, kD));
  
    this.accelerometer = accelerometer;

    getController().setTolerance(Constants.GYRO_TOLERANCE);
    setSetpoint(0);

    enable();
  }

  public double getOutput(){
    return output;
  }

  public void resetMax(){
    maxXAccel = 0;
    maxYAccel = 0;
    maxZAccel = 0;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    this.output = output;
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here

    double xAccel = accelerometer.getX();
    double yAccel = accelerometer.getY();
    double zAccel = accelerometer.getZ();

    maxXAccel = Math.max(xAccel, maxXAccel);
    maxYAccel = Math.max(yAccel, maxYAccel);
    maxZAccel = Math.max(zAccel, maxZAccel);

    SmartDashboard.putNumber("X Accel", xAccel);
    SmartDashboard.putNumber("Y Accel", yAccel);
    SmartDashboard.putNumber("Z Accel", zAccel);

    SmartDashboard.putNumber("Max X Accel", maxXAccel);
    SmartDashboard.putNumber("Max Y Accel", maxYAccel);
    SmartDashboard.putNumber("Max Z Accel", maxZAccel);
    
    return zAccel;
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }
}
