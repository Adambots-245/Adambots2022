/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Log;

public class GondolaSubsystem extends SubsystemBase {
  /**
   * Creates a new GondolaSubsystem.
   */
  public WPI_VictorSPX gondolaMotor;

  public GondolaSubsystem(WPI_VictorSPX gondolaMotor) {
    super();

    this.gondolaMotor = gondolaMotor; //new WPI_VictorSPX(Constants.CLIMBING_GONDOLA_ADJUSTMENT_MOTOR_PORT);
    Log.info("Initializing Gondola");
  }

  public void gondola(double speed) {
    //Log.infoF("Operating gondola - Speed: %f", speed);
    gondolaMotor.set(ControlMode.PercentOutput, speed);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
