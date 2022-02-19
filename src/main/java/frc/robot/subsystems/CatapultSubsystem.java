/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Log;
public class CatapultSubsystem extends SubsystemBase {
  /**
   * Creates a new CatapultSubsystem.
   */

  private WPI_VictorSPX catapultMotor;
  private DigitalInput chooChooLimitSwitch;
  private DigitalInput bandLimitSwitch;
  private Boolean prevLimitSwitchState = false;

  public CatapultSubsystem(WPI_VictorSPX catapultMotor, DigitalInput chooChooLimitSwitch, DigitalInput bandLimitSwitch) {
    super();

    this.catapultMotor = catapultMotor;
    this.chooChooLimitSwitch = chooChooLimitSwitch;
    this.bandLimitSwitch = bandLimitSwitch;
    Log.info("Initializing Catapult");
  }

  public void catapultMotor(double speed) {
    catapultMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    if (chooChooLimitSwitch.get() == true && prevLimitSwitchState == false) {
      catapultMotor.set(ControlMode.PercentOutput, 0);
    }

    prevLimitSwitchState = chooChooLimitSwitch.get();
  }
}
