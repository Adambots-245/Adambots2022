/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.PhotoEye;
import frc.robot.utils.Log;

public class ConveyorSubsystem extends SubsystemBase {
  /**
   * Creates a new ConveyorSubsystem.
   */
  private WPI_VictorSPX conveyorMotor;
  private WPI_VictorSPX alignmentBeltMotor;

  private PhotoEye intakePhotoEye;
  private PhotoEye spacingPhotoEye;
  private PhotoEye finalPhotoEye;

  // private boolean didSpacingEyePreviouslyDetectBall = false;
  // private int ballsPresent = 3;

  public ConveyorSubsystem(WPI_VictorSPX conveyorMotor, WPI_VictorSPX alignmentBeltMotor, PhotoEye intakePhotoEye, PhotoEye spacingPhotoEye, PhotoEye finalPhotoEye) {
    super();
    
    this.conveyorMotor = conveyorMotor; //new WPI_VictorSPX(Constants.INFEED_CONVEYOR_MOTOR_PORT);
    this.alignmentBeltMotor = alignmentBeltMotor; //new WPI_VictorSPX(Constants.INFEED_CONVEYOR_INDEXER_MOTOR_PORT);

    this.intakePhotoEye = intakePhotoEye;
    this.spacingPhotoEye = spacingPhotoEye;
    this.finalPhotoEye = finalPhotoEye;

    Log.info("Initializing Conveyor");
  }

  public boolean allSensorsDetected() {
    System.out.println("Intake: " + intakePhotoEye.isDetecting());
    System.out.println("Spacing: " + spacingPhotoEye.isDetecting());
    System.out.println("Final: " + finalPhotoEye.isDetecting());
    return intakePhotoEye.isDetecting() && spacingPhotoEye.isDetecting() && finalPhotoEye.isDetecting();
  }

  public void runConveyor(double speed, boolean override){

    //Log.infoF("Running coveyor. %% Output: %f", speed);
    if(!override){
      // if(ballsPresent == 5 || (spacingPhotoEye.isDetecting() && !didSpacingEyePreviouslyDetectBall)){
      //   stopConveyorMotor();

      //   if (spacingPhotoEye.isDetecting())
      //     didSpacingEyePreviouslyDetectBall = true;
      // } else {

      //   if(intakePhotoEye.isDetecting()){
      //     conveyorMotor.set(ControlMode.PercentOutput, speed);
      //   }
      // }

        if (allSensorsDetected()) {
          stopConveyorMotor();
        }
        else {
          conveyorMotor.set(ControlMode.PercentOutput, speed);
        }

    }
    else{
      conveyorMotor.set(ControlMode.PercentOutput, speed);
    }

    //Log.infoF("Running conveyor. %% Output: %f", speed);
  }

  public void stopConveyorMotor(){

    conveyorMotor.set(ControlMode.PercentOutput, Constants.STOP_MOTOR_SPEED);

  }

  public void runAlignmentBelt(double speed){

    //Log.infoF("Running Alignment Belt. %% Output: %f", speed);
    alignmentBeltMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // ballsPresent = 3 + (RobotMap.IntakeCounter.get() - RobotMap.ExitCounter.get()) / 2;
    // if (didSpacingEyePreviouslyDetectBall){
    //   if (!spacingPhotoEye.isDetecting()){
    //     didSpacingEyePreviouslyDetectBall = false;
    //   }
    // }
    
    // SmartDashboard.putNumber("Balls Count", ballsPresent);
    SmartDashboard.putBoolean("All Sensors Detecting", allSensorsDetected());
  }
}
