/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Log;

public class IntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new Intake.
   */

  private BaseMotorController intakeMotor;
 

  public IntakeSubsystem(BaseMotorController intakeMotor) {
    super();
    
    this.intakeMotor = intakeMotor; 
    this.intakeMotor.setInverted(true);

    Log.info("Initializing Intake Subsystem");
  }

  public void intake(double speed) {

    Log.infoF("Intake - Speed: %f", speed);
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void outtake() {
    Log.infoF("Outake - Speed: %f", Constants.OUTTAKE_SPEED);
    intakeMotor.set(ControlMode.PercentOutput, Constants.OUTTAKE_SPEED);
  }

  public void stop(){
    Log.info("Stopping Intake Motor");
    intakeMotor.set(ControlMode.PercentOutput, Constants.STOP_MOTOR_SPEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
