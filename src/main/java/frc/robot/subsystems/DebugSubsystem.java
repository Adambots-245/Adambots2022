package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.PhotoEye;

public class DebugSubsystem extends SubsystemBase{

  private PhotoEye rungArmRetractedSwitch;
  
  private ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
 // private  NetworkTableEntry test = debugTab.add("Test", 0).getEntry();
  // private  NetworkTableEntry testBool = debugTab.add("Test", true).getEntry();
  BooleanSupplier stringEquals = () -> true;

  private  NetworkTableEntry testBool = debugTab.addBoolean("Test", stringEquals));




  public DebugSubsystem() {
  }

  public void initialize() {
     
  }

  @Override
  public void periodic() {
      testBool.setBoolean(rungArmRetractedSwitch.isDetecting());
      //test.setDouble(4.0);
  }
}
