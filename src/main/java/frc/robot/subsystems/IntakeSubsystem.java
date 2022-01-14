/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Log;

public class IntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new Intake.
   */

  private DoubleSolenoid armMover;
  private WPI_VictorSPX intakeMotor;
  private WPI_VictorSPX conveyorMotor;
  private WPI_VictorSPX conveyorIndexerMotor;
  private WPI_VictorSPX feedToBlasterMotor;

  public IntakeSubsystem(DoubleSolenoid armMover, WPI_VictorSPX intakeMotor, WPI_VictorSPX feedToBlasterMotor) {
    super();
    
    this.armMover = armMover; //new DoubleSolenoid(Constants.RAISE_POWER_CELL_INTAKE_SOL_PORT, Constants.LOWER_POWER_CELL_INTAKE_SOL_PORT); // raise = kforward lower = kreverse
    this.intakeMotor = intakeMotor; //new WPI_VictorSPX(Constants.INTAKE_MOTOR_PORT);
    this.feedToBlasterMotor = feedToBlasterMotor; //new WPI_VictorSPX(Constants.FEED_TO_BLASTER_MOTOR_PORT);
    this.feedToBlasterMotor.setInverted(true);
    this.intakeMotor.setInverted(true);

    Log.info("Initializing Intake Subsystem");
  }

  public void intake(double speed) {

    //Log.infoF("Intake - Speed: %f", speed);
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void outtake() {
    //Log.infoF("Outake - Speed: %f", Constants.OUTTAKE_SPEED);
    intakeMotor.set(ControlMode.PercentOutput, Constants.OUTTAKE_SPEED);
  }

  public void conveyor(double conveyorSpeed) {
    //Log.infoF("Conveyor - Speed: %f", conveyorSpeed);
    conveyorMotor.set(ControlMode.PercentOutput, conveyorSpeed);
  }

  public void indexToConveyor(double indexToConveyorSpeed) {
    //Log.infoF("IndexToConveyor - Speed: %f", indexToConveyorSpeed);
    conveyorIndexerMotor.set(ControlMode.PercentOutput, indexToConveyorSpeed);
  }

  public void feedToBlaster() {
    //Log.infoF("Feed to Blaster - Speed: %f", Constants.FEED_TO_BLASTER_SPEED);
    feedToBlasterMotor.set(ControlMode.PercentOutput, Constants.FEED_TO_BLASTER_SPEED);
  }

  public void reverseFeedToBlaster(){
    //Log.infoF("Reverse Feed to Blaster - Speed: %f", -Constants.FEED_TO_BLASTER_SPEED);
    feedToBlasterMotor.set(ControlMode.PercentOutput, -Constants.FEED_TO_BLASTER_SPEED);
  }

  public void stopIndex() {
    //Log.infoF("Stop Index - Speed: %f", Constants.STOP_MOTOR_SPEED);
    feedToBlasterMotor.set(ControlMode.PercentOutput, Constants.STOP_MOTOR_SPEED);
  }

  public void stop(){
    Log.info("Stopping Intake Motor");
    intakeMotor.set(ControlMode.PercentOutput, Constants.STOP_MOTOR_SPEED);
  }

  public void RaiseIntake() {
      // Supposedly, if(D-Pad up is pressed)
      //if (GamepadConstants.AXIS_DPAD_POV==true)
      // lol I tried
    Log.info("Raising Intake");
    armMover.set(Value.kForward);     
  }
  
  public void LowerIntake(){
      //Supposedly, if(D-Pad down is pressed)
      //if (GamepadConstants.BUTTON_RB==false)
      //lol I tried
    Log.info("Lowering Intake");
    armMover.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
