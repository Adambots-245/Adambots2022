package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class DebugBoard {

  private static final int ROW1 = 0;
  private static final int ROW2 = 1;
  private static final int ROW3 = 2;
  private static final int ROW4 = 3;
  private static final int ROW5 = 4;
  private static final int COL1 = 0;
  private static final int COL2 = 2;
  private static final int COL3 = 4;
  private static final int COL4 = 6;
  private static final int COL5 = 8;

  private static ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
  
  // prevent instantiation
  private DebugBoard() {
  
  }
  
  public static void  setUpTab() {
    debugTab.addBoolean("Band Limit - Port " + Constants.BAND_HOME_LIMIT_SWITCH_PORT, RobotMap.bandHomeSwitch::get).withSize(2, 1).withPosition(COL5, ROW1);

    debugTab.add("ChooChoo Motor - Port " + Constants.CHOO_CHOO_MOTOR_PORT, RobotMap.ChooChooMotor).withSize(2, 1).withPosition(COL2, ROW3);
    debugTab.add("Band Motor - Port " + Constants.BAND_MOTOR_PORT, RobotMap.BandMotor).withSize(2, 1).withPosition(COL2, ROW4);

    debugTab.addBoolean("Intake Photo Eye  -  Port", RobotMap.intakePhotoEye::isDetecting).withSize(2, 1).withPosition(COL3, ROW3);
    debugTab.addBoolean("Intake Catapult Photo Eye", RobotMap.intakeCatapultPhotoEye::isDetecting).withSize(2, 1).withPosition(COL3, ROW4);
    
    debugTab.addBoolean("GearShifter - Port " + Constants.HIGH_GEAR_SOL_PORT, RobotMap.GearShifter::isDisabled).withSize(2, 1).withPosition(COL5, ROW2);
    
    debugTab.add("Back Left Motor - Port " + Constants.BL_TALON, RobotMap.BackLeftMotor).withSize(2, 1).withPosition(COL1, ROW4);
    debugTab.add("Back Right Motor - Port " + Constants.BR_TALON, RobotMap.BackRightMotor).withSize(2, 1).withPosition(COL1, ROW2);
    debugTab.add("Front Left Motor - Port " + Constants.FL_TALON, RobotMap.FrontLeftMotor).withSize(2, 1).withPosition(COL1, ROW3);
    debugTab.add("Front Right Motor - Port " + Constants.FR_TALON, RobotMap.FrontRightMotor).withSize(2, 1).withPosition(COL1, ROW1);

    debugTab.add("Winch 1 - Port " + Constants.HANG_WINCH_1_MOTOR_PORT, RobotMap.winchMotor1).withSize(2, 1).withPosition(COL2, ROW1);
    debugTab.add("Winch 2 - Port " + Constants.HANG_WINCH_2_MOTOR_PORT, RobotMap.winchMotor2).withSize(2, 1).withPosition(COL2, ROW2);

    debugTab.addBoolean("L Clamp Switch - Port " + Constants.LEFT_CLAMP_LIMIT_SWITCH_PORT, RobotMap.leftClampSwitch::get).withSize(2, 1).withPosition(COL3, ROW1);
    debugTab.addBoolean("R Clamp Switch - Port " + Constants.RIGHT_CLAMP_LIMIT_SWITCH_PORT, RobotMap.rightClampSwitch::get).withSize(2, 1).withPosition(COL3, ROW2);

    debugTab.addBoolean("L Rung Switch - Port " + Constants.LEFT_RUNG_LIMIT_SWITCH_PORT, RobotMap.leftRungSwitch::get).withSize(2, 1).withPosition(COL4, ROW4);
    debugTab.addBoolean("R Rung Switch - Port " + Constants.RIGHT_RUNG_LIMIT_SWITCH_PORT, RobotMap.rightRungSwitch::get).withSize(2, 1).withPosition(COL4, ROW5);
    
    debugTab.addBoolean("Retracted Switch - Port " + Constants.RUNG_ARM_RETRACTED_PHOTO_EYE_PORT, RobotMap.rungArmRetractedSwitch::isDetecting).withSize(2, 1).withPosition(COL4, ROW1);
    debugTab.addBoolean("Mid Switch - Port " + Constants.RUNG_ARM_MID_PHOTO_EYE_PORT, RobotMap.rungArmMidSwitch::isDetecting).withSize(2, 1).withPosition(COL4, ROW2);
    debugTab.addBoolean("Advanced Switch - Port " + Constants.RUNG_ARM_ADVANCED_PHOTO_EYE_PORT, RobotMap.rungArmAdvancedSwitch::isDetecting).withSize(2, 1).withPosition(COL4, ROW3);
    

    debugTab.addBoolean("Ring Light - Port " + Constants.RING_LIGHT_PORT, RobotMap.RingLight::isDisabled).withSize(2, 1).withPosition(COL5, ROW3);
  }
}
