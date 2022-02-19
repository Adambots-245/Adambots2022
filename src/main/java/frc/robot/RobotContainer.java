
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Gamepad.Buttons;
import frc.robot.Gamepad.GamepadConstants;

import frc.robot.commands.*;
import frc.robot.commands.autonCommands.*;
import frc.robot.commands.autonCommands.autonCommandGroups.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // subsystems
  private final CatapultSubsystem catapultSubsystem = new CatapultSubsystem(RobotMap.ChooChooMotor, RobotMap.ChooChooLimitSwitch, RobotMap.BandMotor, RobotMap.ChooChooLimitSwitch);
  
  // commands
  private BackboardToggleCommand backboardToggleCommand;
  private ConveyorCommand conveyorCommand;
  private DriveForwardDistanceCommand autonDriveForwardDistanceCommand;
  private TurnToAngleCommand autonTurn90DegreeCommand;
  private GondolaCommand gondolaCommand;
  private GyroDriveForDistCommand autonGyroDriveForwardDistanceCommand;
  private RaiseElevatorCommand raiseElevatorCommand;
  private SequentialCommandGroup autonDriveForwardGyroDistanceCommand;
  private WinchCommand winchCommand;
  
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    setupDefaultCommands();

    // Log.saveToFile("/home/lvuser/robot.txt");

    // Configure the button bindings
    configureButtonBindings();

    // configure the dashboard
    dash();

    // auton commands
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    // primary controls
      
    Buttons.primaryXButton.whenPressed(new CatapultFireCommand(catapultSubsystem));
    Buttons.primaryYButton.whenPressed(new BandMoveCommand(catapultSubsystem, 0.5));
    Buttons.primaryAButton.whenPressed(new BandMoveCommand(catapultSubsystem, -0.5));
      
  }

  private void dash(){

  }

  private void setupDefaultCommands(){
 
  }

  // deadzoning
  private double deaden(double rawInput) {
    return Math.abs(rawInput) < GamepadConstants.DEADZONE ? 0 : rawInput;
  }

  /**
   * Use this to pass the autonomo
   * us command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    System.out.println(autoChooser.getSelected());
    return autoChooser.getSelected();
  }
}
