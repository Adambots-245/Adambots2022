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
    debugTab.add("Band Limit - Port " + Constants.BAND_HOME_LIMIT_SWITCH_PORT, RobotMap.bandHomeSwitch).withSize(2, 1);

    // debugTab.add("Catapult Stop" + Constants.RAISE_CATAPULT_STOP_SOL_PORT, RobotMap.CatapultStop);
    debugTab.add("ChooChoo Motor - Port " + Constants.CHOO_CHOO_MOTOR_PORT, RobotMap.ChooChooMotor).withSize(2, 1);
    debugTab.add("Band Motor - Port " + Constants.BAND_MOTOR_PORT, RobotMap.BandMotor).withSize(2, 1);

    // debugTab.add("Gyro Sensor", (Sendable) Gyro.getInstance()).withWidget(BuiltInWidgets.kGyro);

    // debugTab.add("Accelerometer", RobotMap.Accelerometer);

    debugTab.addBoolean("Intake Photo Eye  -  Port", RobotMap.intakePhotoEye::isDetecting).withSize(2, 1);
    debugTab.addBoolean("Intake Catapult Photo Eye", RobotMap.intakeCatapultPhotoEye::isDetecting).withSize(2, 1);
    
    debugTab.addBoolean("GearShifter - Port " + Constants.HIGH_GEAR_SOL_PORT, RobotMap.GearShifter::isDisabled).withSize(2, 1);
    
    debugTab.add("Back Left Motor - Port " + Constants.BL_TALON, RobotMap.BackLeftMotor).withSize(2, 1);
    debugTab.add("Back Right Motor - Port " + Constants.BR_TALON, RobotMap.BackRightMotor).withSize(2, 1);
    debugTab.add("Front Left Motor - Port " + Constants.FL_TALON, RobotMap.FrontLeftMotor).withSize(2, 1);
    debugTab.add("Front Right Motor - Port " + Constants.FR_TALON, RobotMap.FrontRightMotor).withSize(2, 1);

    // port?
    //debugTab.add("Color Sensor - Port " + Constants.)
    debugTab.add("Winch 1 - Port " + Constants.HANG_WINCH_1_MOTOR_PORT, RobotMap.winchMotor1).withSize(2, 1);
    debugTab.add("Winch 2 - Port " + Constants.HANG_WINCH_2_MOTOR_PORT, RobotMap.winchMotor2).withSize(2, 1);

    // debugTab.addBoolean("Intake Phot Eye", )

    // debugTab.add("Hang Clamp - Port " + Constants.clamp)

    
    
    debugTab.addBoolean("L Clamp Switch - Port " + Constants.LEFT_CLAMP_LIMIT_SWITCH_PORT, RobotMap.leftClampSwitch::get).withSize(2, 1);
    debugTab.addBoolean("R Clamp Switch - Port " + Constants.RIGHT_CLAMP_LIMIT_SWITCH_PORT, RobotMap.rightClampSwitch::get).withSize(2, 1);

    debugTab.addBoolean("L Rung Switch - Port " + Constants.LEFT_RUNG_LIMIT_SWITCH_PORT, RobotMap.leftRungSwitch::get).withSize(2, 1);
    debugTab.addBoolean("R Rung Switch - Port " + Constants.RIGHT_RUNG_LIMIT_SWITCH_PORT, RobotMap.rightRungSwitch::get).withSize(2, 1);
    
    debugTab.addBoolean("Retracted Switch - Port " + Constants.RUNG_ARM_RETRACTED_PHOTO_EYE_PORT, RobotMap.rungArmRetractedSwitch::isDetecting).withSize(2, 1).withPosition(0, 0).withSize(2, 1);
    debugTab.addBoolean("Mid Switch - Port " + Constants.RUNG_ARM_MID_PHOTO_EYE_PORT, RobotMap.rungArmMidSwitch::isDetecting).withSize(2, 1);
    debugTab.addBoolean("Advanced Switch - Port " + Constants.RUNG_ARM_ADVANCED_PHOTO_EYE_PORT, RobotMap.rungArmAdvancedSwitch::isDetecting).withSize(2, 1);
    

    debugTab.addBoolean("Ring Light - Port " + Constants.RING_LIGHT_PORT, RobotMap.RingLight::isDisabled).withSize(2, 1);

    

  //  debugTab.add("", RobotMap.)


  }

  @Override
  public void periodic() {
      // testBool.setBoolean(rungArmRetractedSwitch.isDetecting());
      //test.setDouble(4.0);
  }
}
