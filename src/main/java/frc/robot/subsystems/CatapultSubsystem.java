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
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

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

  public CatapultSubsystem(BaseMotorController catapultMotor, DigitalInput chooChooLimitSwitch, BaseMotorController bandMotor, DigitalInput bandLimitSwitch) {
    super();

    this.catapultMotor = catapultMotor;
    this.bandMotor = bandMotor;
    this.chooChooLimitSwitch = chooChooLimitSwitch;
    this.bandLimitSwitch = bandLimitSwitch;
    Log.info("Initializing Catapult");

    initialize();
  }

  public void catapultMotor(double speed) {
    catapultMotor.set(ControlMode.PercentOutput, speed);
  }

  public void bandMotor(double speed) {
    if (speed < 0 && bandLimitSwitch.get()) {
      speed = 0;
    }
    bandMotor.set(ControlMode.PercentOutput, speed);
  }

  public void initialize() {
    bandMotor.setNeutralMode(NeutralMode.Brake);
    catapultMotor.setNeutralMode(NeutralMode.Brake);

    bandMotor.set(ControlMode.PercentOutput, -0.5);
  }

  @Override
  public void periodic() {
    if (chooChooLimitSwitch.get() == true && prevChooChooLimitSwitchState == false) {
      catapultMotor.set(ControlMode.PercentOutput, 0);
    }
    if (bandLimitSwitch.get() == true && prevBandLimitSwitchState == false) {
      bandMotor.set(ControlMode.PercentOutput, 0);
      bandMotor.setSelectedSensorPosition(0);
    }

    prevChooChooLimitSwitchState = chooChooLimitSwitch.get();
    prevBandLimitSwitchState = bandLimitSwitch.get();

    //System.out.println(bandMotor.getSelectedSensorPosition());
  }
}
