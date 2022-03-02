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
  private Solenoid catapultStop;

  private Boolean ChooChooLimitSwitchState = false;
  private Boolean prevChooChooLimitSwitchState = false;
  private Boolean enableBand = false;
  private Boolean prevBandLimitSwitchState = false;

  private double bandTarget = 0;
  private double error = 0;
  private int accumulate = 0;

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

  public void initialize() {
    catapultMotor.setNeutralMode(NeutralMode.Brake);
    catapultMotor.setInverted(true);
    accumulate = 0;

    bandMotor.setNeutralMode(NeutralMode.Brake);
    bandMotor.setSelectedSensorPosition(0);
    bandMotor.set(ControlMode.PercentOutput, -0.5);
    enableBand = false;
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

  public void bandMotor() {
    error = (bandTarget - bandMotor.getSelectedSensorPosition());
    double motorCommand = 0;
    if (Math.abs(error) > Constants.ACCEPTABLE_BAND_ERROR) {
      if (error < 0) {motorCommand = Constants.MAX_BAND_MOVE_SPEED;}
      else {motorCommand = -Constants.MAX_BAND_MOVE_SPEED;}
    }
    else { //Zeros error if we hit the target
      bandTarget = bandMotor.getSelectedSensorPosition();
    }

    bandMotor.set(ControlMode.PercentOutput, motorCommand);
  }

  public void raiseStop() {
    catapultStop.set(true);
  }

  public void lowerStop(){
    catapultStop.set(false);
  }

  public void runBandMotor(double speed){
    bandMotor.set(ControlMode.PercentOutput, speed);
  }

  public void accumulateLogic () {
    if (chooChooLimitSwitch.get()) {
      accumulate = Math.min(accumulate+1, 5);
    }
    else {
      accumulate = Math.max(accumulate-1, -5);
    }
    ChooChooLimitSwitchState = (accumulate >= 5);
  }

  @Override
  public void periodic() {
    accumulateLogic();

    if (ChooChooLimitSwitchState == true && prevChooChooLimitSwitchState == false) { //Testing if Choo Choo limit switch goes from low -> high and stopping the motor
      catapultMotor.set(ControlMode.PercentOutput, 0);
    }

    // Temporarily commenting it since band limit switch is always true
    //if (bandLimitSwitch.get() == true && prevBandLimitSwitchState == false) { //Testing if Band limit switch goes from low -> high and stopping the motor + zeroing encoder
    //  bandMotor.set(ControlMode.PercentOutput, 0);
    //  bandMotor.setSelectedSensorPosition(0);
      enableBand = true;
    //}

    if (enableBand == true) {bandMotor();}

    prevChooChooLimitSwitchState = ChooChooLimitSwitchState;
    prevBandLimitSwitchState = bandLimitSwitch.get();

    System.out.println("Current Pos: " + bandMotor.getSelectedSensorPosition() + " | Error: " + error);
  }
}
