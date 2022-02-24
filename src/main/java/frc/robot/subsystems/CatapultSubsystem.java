/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Log;

public class CatapultSubsystem extends SubsystemBase {
  /**
   * Creates a new CatapultSubsystem.
   */

  private BaseMotorController catapultMotor;
  private BaseMotorController bandMotor;
  private DigitalInput chooChooLimitSwitch;
  private DigitalInput bandLimitSwitch;
  private Boolean prevChooChooLimitSwitchState = false;
  private Boolean prevBandLimitSwitchState = false;
  private Double bandTarget = 0D;
  private Boolean enableBand = false;

  public CatapultSubsystem(BaseMotorController catapultMotor, DigitalInput chooChooLimitSwitch, BaseMotorController bandMotor, DigitalInput bandLimitSwitch) {
    super();

    this.catapultMotor = catapultMotor;
    this.bandMotor = bandMotor;
    this.chooChooLimitSwitch = chooChooLimitSwitch;
    this.bandLimitSwitch = bandLimitSwitch;
    Log.info("Initializing Catapult");

    initialize();
  }

  public void setCatapultMotor(double speed) {
    catapultMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setBandTarget(Double target) {
    bandTarget = target;
  }

  public void bandMotor() {
    double error = (bandTarget - bandMotor.getSelectedSensorPosition())/3000;
    error = Math.min(error, 0.3);
    error = Math.max(error, -0.3);

    bandMotor.set(ControlMode.PercentOutput, error);
  }

  public void initialize() {
    bandMotor.setNeutralMode(NeutralMode.Brake);
    catapultMotor.setNeutralMode(NeutralMode.Brake);

    bandMotor.set(ControlMode.PercentOutput, -0.5);
    enableBand = false;
  }

  @Override
  public void periodic() {
    if (chooChooLimitSwitch.get() == true && prevChooChooLimitSwitchState == false) { //Testing if Choo Choo limit switch goes from low -> high and stopping the motor
      catapultMotor.set(ControlMode.PercentOutput, 0);
    }

    if (bandLimitSwitch.get() == true && prevBandLimitSwitchState == false) { //Testing if Band limit switch goes from low -> high and stopping the motor + zeroing encoder
      bandMotor.set(ControlMode.PercentOutput, 0);
      bandMotor.setSelectedSensorPosition(0);
      enableBand = true;
    }

    if (enableBand == true) {bandMotor();}

    prevChooChooLimitSwitchState = chooChooLimitSwitch.get();
    prevBandLimitSwitchState = bandLimitSwitch.get();
  }
}