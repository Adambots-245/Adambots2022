package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
    debugTab.add("ChooChoo Motor", RobotMap.ChooChooMotor);
    debugTab.add("Left Clamp Switch", RobotMap.leftClampedSwitch).withWidget(BuiltInWidgets.kBooleanBox);
    debugTab.addBoolean("Right Clamp Switch", RobotMap.rightClampedSwitch::get);
  }

  @Override
  public void periodic() {
      // testBool.setBoolean(rungArmRetractedSwitch.isDetecting());
      //test.setDouble(4.0);
  }
}
