package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.sensors.Gyro;
import frc.robot.sensors.PhotoEye;

public class DebugSubsystem extends SubsystemBase{

  private PhotoEye rungArmRetractedSwitch;
  
  private ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
 // private  NetworkTableEntry test = debugTab.add("Test", 0).getEntry();
  // private  NetworkTableEntry testBool = debugTab.add("Test", true).getEntry();
  // BooleanSupplier stringEquals = () -> true;

  
  
  
  public DebugSubsystem() {
    initialize();
  }
  
  public void initialize() {

    debugTab.add("Catapult Stop" + Constants.RAISE_CATAPULT_STOP_SOL_PORT, RobotMap.CatapultStop);
    debugTab.add("ChooChoo Motor - Port " + Constants.CHOO_CHOO_MOTOR_PORT, RobotMap.ChooChooMotor);
    debugTab.add("Band Motor - Port " + Constants.BAND_MOTOR_PORT, RobotMap.BandMotor);

    // debugTab.add("Gyro Sensor", (Sendable) Gyro.getInstance()).withWidget(BuiltInWidgets.kGyro);

    // debugTab.add("Accelerometer", RobotMap.Accelerometer);

    // debugTab.addBoolean("Intake Photo Eye", RobotMap.intakePhotoEye::isDetecting);
    // debugTab.addBoolean("Intake Catapult Photo Eye", RobotMap.intakeCatapultPhotoEye::isDetecting);
    
    debugTab.addBoolean("GearShifter - Port " + Constants.HIGH_GEAR_SOL_PORT, RobotMap.GearShifter::isDisabled);
    
    debugTab.add("Back Left Motor - Port " + Constants.BL_TALON, RobotMap.BackLeftMotor);
    debugTab.add("Back Right Motor - Port " + Constants.BR_TALON, RobotMap.BackRightMotor);
    debugTab.add("Front Left Motor - Port " + Constants.FL_TALON, RobotMap.FrontLeftMotor);
    debugTab.add("Front Right Motor - Port " + Constants.FR_TALON, RobotMap.FrontRightMotor);

    // port?
    //debugTab.add("Color Sensor - Port " + Constants.)
    debugTab.add("Winch 1 - Port " + Constants.HANG_WINCH_1_MOTOR_PORT, RobotMap.winchMotor1);
    debugTab.add("Winch 2 - Port " + Constants.HANG_WINCH_2_MOTOR_PORT, RobotMap.winchMotor2);

    // debugTab.addBoolean("Intake Phot Eye", )

    // debugTab.add("Hang Clamp - Port " + Constants.clamp)

    
    
    debugTab.addBoolean("L Clamp Switch - Port " + Constants.LEFT_CLAMP_LIMIT_SWITCH_PORT, RobotMap.leftClampSwitch::get);
    debugTab.addBoolean("R Clamp Switch - Port " + Constants.RIGHT_CLAMP_LIMIT_SWITCH_PORT, RobotMap.rightClampSwitch::get);

    debugTab.addBoolean("L Rung Switch - Port " + Constants.LEFT_RUNG_LIMIT_SWITCH_PORT, RobotMap.leftRungSwitch::get);
    debugTab.addBoolean("R Rung Switch - Port " + Constants.RIGHT_RUNG_LIMIT_SWITCH_PORT, RobotMap.rightRungSwitch::get);
    
    debugTab.addBoolean("Retracted Switch - Port " + Constants.RUNG_ARM_RETRACTED_PHOTO_EYE_PORT, RobotMap.rungArmRetractedSwitch::isDetecting).withSize(10, 10);
    debugTab.addBoolean("Mid Switch - Port " + Constants.RUNG_ARM_MID_PHOTO_EYE_PORT, RobotMap.rungArmMidSwitch::isDetecting);
    debugTab.addBoolean("Advanced Switch - Port " + Constants.RUNG_ARM_ADVANCED_PHOTO_EYE_PORT, RobotMap.rungArmAdvancedSwitch::isDetecting);
    

    debugTab.addBoolean("Ring Light - Port " + Constants.RING_LIGHT_PORT, RobotMap.RingLight::isDisabled);

    

  //  debugTab.add("", RobotMap.)


  }

  @Override
  public void periodic() {
      // testBool.setBoolean(rungArmRetractedSwitch.isDetecting());
      //test.setDouble(4.0);
  }
}
