/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CatapultSubsystem;

public class CatapultAutoDistanceFireCommand extends CommandBase {
  /**
   * Creates a new Command for testing.
   */

  private final CatapultSubsystem catapultSubsystem;
  private NetworkTableEntry distanceEntry;

  public CatapultAutoDistanceFireCommand(CatapultSubsystem catapultSubsystem) {
    this.catapultSubsystem = catapultSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(catapultSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable table = instance.getTable(Constants.VISION_TABLE_NAME);
    distanceEntry = table.getEntry(Constants.HUB_DISTANCE_ENTRY_NAME);

    double target = distanceEntry.getDouble(420)/1; //Callibrate formula to convert from distance to band tension encoder ticks

    if (distanceEntry.exists()) { //Only worry about moving the bands if the entry actually exists
      catapultSubsystem.setEncoderMode(true);
      catapultSubsystem.setBandTarget(target);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new CatapultFireCommand(catapultSubsystem);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Terminate once we have reached the band target or if there is no distance entry
    return Math.abs(catapultSubsystem.getError()) < Constants.ACCEPTABLE_BAND_ERROR || !distanceEntry.exists(); 
  }
}
