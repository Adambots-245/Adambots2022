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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
  private Solenoid catapultStop;
  private int accumulate = 0;
  private boolean limitSwitch = false;

  public double error = 0;

  public CatapultSubsystem(BaseMotorController catapultMotor, DigitalInput chooChooLimitSwitch, BaseMotorController bandMotor, DigitalInput bandLimitSwitch, Solenoid catapultStop) {
    super();

    this.catapultMotor = catapultMotor;
    this.bandMotor = bandMotor;
    this.chooChooLimitSwitch = chooChooLimitSwitch;
    this.bandLimitSwitch = bandLimitSwitch;
    this.catapultStop = catapultStop;
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
    // error = (bandTarget - bandMotor.getSelectedSensorPosition())/3000; //Arbitrary sensitivity value, adjust when we have robot
    // error = Math.min(error, Constants.MAX_BAND_MOVE_SPEED);
    // error = Math.max(error, -Constants.MAX_BAND_MOVE_SPEED);

    // bandMotor.set(ControlMode.PercentOutput, error);
  }

  public void RaiseStop() {
    catapultStop.set(true);
  }

  public void LowerStop(){
    catapultStop.set(false);
  }

  public void runBandMotor(double speed){
    //System.out.println("Band Motor: " + speed);
    bandMotor.set(ControlMode.PercentOutput, speed);
  }

  public void initialize() {
    bandMotor.setNeutralMode(NeutralMode.Brake);
    catapultMotor.setNeutralMode(NeutralMode.Brake);
    catapultMotor.setInverted(true);

    accumulate = 0;
    //bandMotor.set(ControlMode.PercentOutput, -0.5);
    enableBand = false;
  }

  @Override
  public void periodic() {
    if (chooChooLimitSwitch.get()) {
      accumulate = Math.min(accumulate+1, 5);
    }
    else {
      accumulate = Math.max(accumulate-1, -5);
    }
    if (accumulate >= 5) {
      limitSwitch = true;
    }
    else {
      limitSwitch = false;
    }

    // System.out.println("Band Motor Pos: " + bandMotor.getSelectedSensorPosition() + " - Band Limit Switch: " + bandLimitSwitch.get());

    System.out.println("Choo Choo: " + limitSwitch + " Accumulate: " + accumulate);
    if (limitSwitch == true && prevChooChooLimitSwitchState == false) { //Testing if Choo Choo limit switch goes from low -> high and stopping the motor
      catapultMotor.set(ControlMode.PercentOutput, 0);
    }

    /** Temporarily commenting it since band limit switch is always true
    if (bandLimitSwitch.get() == true && prevBandLimitSwitchState == false) { //Testing if Band limit switch goes from low -> high and stopping the motor + zeroing encoder
      bandMotor.set(ControlMode.PercentOutput, 0);
      bandMotor.setSelectedSensorPosition(0);
      enableBand = true;
    }
    **/

    //if (enableBand == true) {bandMotor();}

    prevChooChooLimitSwitchState = limitSwitch;
    prevBandLimitSwitchState = bandLimitSwitch.get();
  }
}
