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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private DigitalInput bandHomeLimitSwitch;
  private Solenoid catapultStop;

  private Boolean ChooChooLimitSwitchState = false;
  private Boolean prevChooChooLimitSwitchState = false;
  private Boolean encoderMode = false;

  private double bandTarget = 0;
  private double error = 0;
  private int accumulate = 0;
  private double bandSpeed = 0;

  public CatapultSubsystem(BaseMotorController catapultMotor, DigitalInput chooChooLimitSwitch, DigitalInput bandHomeLimitSwitch, BaseMotorController bandMotor, Solenoid catapultStop) {
    super();

    this.catapultMotor = catapultMotor;
    this.bandMotor = bandMotor;
    this.chooChooLimitSwitch = chooChooLimitSwitch;
    this.bandHomeLimitSwitch = bandHomeLimitSwitch;
    this.catapultStop = catapultStop;

    Log.info("Initializing Catapult");
    initialize();
  }

  public void initialize() {
    catapultMotor.setNeutralMode(NeutralMode.Brake);
    catapultMotor.setInverted(true);
    accumulate = 0;

    bandMotor.setNeutralMode(NeutralMode.Brake);
    bandMotor.setSelectedSensorPosition(-Constants.HOME_TENSION*4096*20);
    encoderMode = false;
  }

  public double getError () {
    return error;
  }

  public void runCatapult(double speed) {
    catapultMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setBandTarget(double target) {
    bandTarget = target;
  }

  public void setEncoderMode(Boolean state) {
    encoderMode = state;
  }

  public boolean getBandSwitch () {
    return bandHomeLimitSwitch.get();
  }

  public boolean getCatapultSwitch () {
    return ChooChooLimitSwitchState;
  }

  public void bandMotor() {
    error = (bandTarget+bandMotor.getSelectedSensorPosition());
    System.out.println(error);
    double motorCommand = 0;
    if (Math.abs(error) > Constants.ACCEPTABLE_BAND_ERROR) {
      if (error < 0) {
        motorCommand = Constants.MAX_BAND_MOVE_SPEED;
      }
      else {
        motorCommand = -Constants.MAX_BAND_MOVE_SPEED;
      }
    }
    bandSpeed = motorCommand;
  }

  public void raiseStop() {
    catapultStop.set(true);
  }

  public void lowerStop(){
    catapultStop.set(false);
  }

  public void runBandMotor(double speed){
    bandSpeed = speed;
  }

  public void accumulateLogic () {
    if (chooChooLimitSwitch.get()) {
      accumulate = Math.min(accumulate+1, 5);
    }
    else {
      accumulate = Math.max(accumulate-1, -5);
    }
    ChooChooLimitSwitchState = (accumulate >= 3);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Band Encoder In", bandMotor.getSelectedSensorPosition()/4096/20);
    SmartDashboard.putNumber("Error", error);

    accumulateLogic();
    if (encoderMode) {bandMotor();}

    if (ChooChooLimitSwitchState == true && prevChooChooLimitSwitchState == false) { //Testing if Choo Choo limit switch goes from low -> high and stopping the motor
      catapultMotor.set(ControlMode.PercentOutput, 0);
    }

    if (bandHomeLimitSwitch.get() == true && bandSpeed > 0) { //Testing if Choo Choo limit switch goes from low -> high and stopping the motor
      bandMotor.setSelectedSensorPosition(0);
      bandSpeed = 0;
      encoderMode = true;
    }
    bandMotor.set(ControlMode.PercentOutput, bandSpeed);

    prevChooChooLimitSwitchState = ChooChooLimitSwitchState;

    //System.out.println("Current Pos: " + bandMotor.getSelectedSensorPosition() + " | Error: " + error);
    // System.out.println("Band Limit Switch: " + bandHomeLimitSwitch.get());
  }
}
