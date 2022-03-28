/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.sensors.PhotoEye;
import frc.robot.utils.Log;

public class IntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new Intake.
   */

  private BaseMotorController intakeMotor;
  private DoubleSolenoid intakeExtend;
  private boolean intakeIsOut;
  private PhotoEye intakeCatapultPhotoEye;
  private PhotoEye intakePhotoEye;
  private double motorSpeed = 0.0;

  private DigitalInput chooChooOpticalSensor;

  private boolean intakeCatapultBool;
  private boolean intakeBool;
  
  // private PhotoEye intakeSwitch;

 

  public IntakeSubsystem(BaseMotorController intakeMotor, DoubleSolenoid intakeExtend, PhotoEye intakePhotoEye, PhotoEye intakeCatapultPhotoEye, DigitalInput chooChooOpticalSensor) {
    super();
    
    this.intakeMotor = intakeMotor; 
    this.intakeExtend = intakeExtend;
    this.intakeMotor.setInverted(true);
    this.intakePhotoEye = intakePhotoEye;
    this.intakeCatapultPhotoEye = intakeCatapultPhotoEye;
    this.chooChooOpticalSensor = chooChooOpticalSensor;
    Log.info("Initializing Intake Subsystem");
  }

  public void intake(double speed) {
    Log.infoF("Intake - Speed: %f", speed);
    motorSpeed = speed;
  }

  public void intakeWithColor(double speed) {
    Log.infoF("Intake - Speed: %f", speed);
    
    Color ballColor = RobotMap.ColorSensor.matchClosestColor(RobotMap.ColorSensor.getColor());
    int direction = 1;

    if (Constants.BALL_COLOR_DETECTION){
      switch (DriverStation.getAlliance()){
        case Red:
          if (ballColor == Color.kBlue)
            direction = -1; // reverse the intake
        case Blue:
          if (ballColor == Color.kRed)
            direction = -1;
        default:
          break;
      }
    }
    
    // intakeMotor.set(ControlMode.PercentOutput, speed * direction);
    motorSpeed = speed * direction;
  }

  public void outtake() {
    Log.infoF("Outake - Speed: %f", Constants.OUTTAKE_SPEED);
    // intakeMotor.set(ControlMode.PercentOutput, Constants.OUTTAKE_SPEED);
    motorSpeed = Constants.OUTTAKE_SPEED;
  }

  public void stop(){
    Log.info("Stopping Intake Motor");
    // intakeMotor.set(ControlMode.PercentOutput, Constants.STOP_MOTOR_SPEED);
    motorSpeed = Constants.STOP_MOTOR_SPEED;
  }

  public void intakeOut(){
    Log.info("Intake out initiated");
    intakeExtend.set(Value.kForward);
    intakeIsOut = true;
  }

  public void intakeIn(){
    Log.info("Intake in initiated");
    intakeExtend.set(Value.kReverse);
    intakeIsOut = false;
  }


  @Override
  public void periodic() {
    intakeCatapultBool = !intakeCatapultPhotoEye.isDetecting();
    intakeBool = !intakePhotoEye.isDetecting();

    // This method will be called once per scheduler run

    // SmartDashboard.putBoolean("intake", intakeSwitch.isDetecting());
    if (intakeBool && intakeCatapultBool && motorSpeed < 0 && chooChooOpticalSensor.get()) {
      stop();
    } 

    if (!intakeIsOut){
      stop();
    }

    intakeMotor.set(ControlMode.PercentOutput, motorSpeed);    
  }
}

