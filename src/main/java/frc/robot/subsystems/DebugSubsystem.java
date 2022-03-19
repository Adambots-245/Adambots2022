package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DebugSubsystem extends SubsystemBase{
  
  private ShuffleboardTab debugTab =Shuffleboard.getTab("Debug");
  private  NetworkTableEntry test=debugTab.add("Test", 0)
  .getEntry();
  

  public DebugSubsystem() {
  }

  public void initialize() {
     
  }

  @Override
  public void periodic() {
      test.setDouble(4.0);
  }
}
