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
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.utils.Log;

public class CatapultSubsystem extends SubsystemBase {
  /**
   * Creates a new CatapultSubsystem.
   */

  public WPI_VictorSPX catapultMotor;

  public CatapultSubsystem(/*WPI_VictorSPX catapultMotor*/) {
    super();

    //this.catapultMotor = catapultMotor; //new WPI_VictorSPX(Constants.CATAPULT_MOTOR_PORT);
    Log.info("Initializing Gondola");
  }

  public void printSomething(String print) {
    System.out.println(print);
  }

  @Override
  public void periodic() {
    //System.out.println("hello");
    //printSomething();
  }
}
