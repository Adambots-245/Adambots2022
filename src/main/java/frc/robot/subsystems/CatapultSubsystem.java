/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Log;

public class CatapultSubsystem extends SubsystemBase {
  /**
   * Creates a new CatapultSubsystem.
   */

  public WPI_VictorSPX catapultMotor;
  private DigitalInput limitSwitch;

  public CatapultSubsystem(WPI_VictorSPX catapultMotor, DigitalInput limitSwitch) {
    super();

    this.catapultMotor = catapultMotor;
    this.limitSwitch = limitSwitch;
    Log.info("Initializing Catapult");
  }

  public void printSomething(String print) {
    System.out.println(print);
  }

  public void catapultMotor(double speed) {
    catapultMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {

  }
}
