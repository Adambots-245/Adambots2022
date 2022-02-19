
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
  private final ControlPanelSubsystem panelSubsystem = new ControlPanelSubsystem(RobotMap.PanelMotor, RobotMap.ColorSensor);
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem(RobotMap.ConveyorMotor, RobotMap.AlignmentBeltMotor, RobotMap.IntakePhotoEye, RobotMap.SpacingPhotoEye, RobotMap.ExitPhotoEye);
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(RobotMap.GyroSensor, RobotMap.GearShifter, RobotMap.FrontRightMotor, RobotMap.FrontLeftMotor, RobotMap.BackLeftMotor, RobotMap.BackRightMotor);
  //private final HangSubsystem hangSubsystem = new HangSubsystem(RobotMap.HangMotor, RobotMap.WinchMotor1, RobotMap.WinchMotor2, RobotMap.LeftLimitSwitch, RobotMap.ChooChooLimitSwitch);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(RobotMap.ArmMover, RobotMap.IntakeMotor, RobotMap.FeedToBlasterMotor);
  private final CatapultSubsystem catapultSubsystem = new CatapultSubsystem(RobotMap.ChooChooMotor, RobotMap.ChooChooLimitSwitch, RobotMap.BandLimitSwitch);
  
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
    driveTrainSubsystem.resetEncoders();

    // configure the dashboard
    dash();

    // auton commands
    autonDriveForwardDistanceCommand = new DriveForwardDistanceCommand(driveTrainSubsystem,
        Constants.AUTON_DRIVE_FORWARD_DISTANCE, Constants.AUTON_DRIVE_FORWARD_SPEED);

    //autonGyroDriveForwardDistanceCommand = new GyroDriveForDistCommand(driveTrainSubsystem,
    //Constants.AUTON_DRIVE_FORWARD_DISTANCE, Constants.AUTON_DRIVE_FORWARD_SPEED, gyroSubsystem.getYaw());
    double autonSpeed = .75;
    autonDriveForwardGyroDistanceCommand = new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.AUTON_PUSH_ROBOT_DISTANCE, autonSpeed*.5, 0, true)
      .andThen(new WaitCommand(1)).andThen(new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.AUTON_FORWARD_BALL_PICKUP_DISTANCE, -autonSpeed, 0, false));
    
    autonTurn90DegreeCommand = new TurnToAngleCommand(driveTrainSubsystem, autonSpeed*0.5, 45, true);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    // primary controls
      Buttons.primaryAButton.whenPressed(new ShiftLowGearCommand(driveTrainSubsystem));
      Buttons.primaryXButton.whenPressed(new CatapultFireCommand(catapultSubsystem));
      Buttons.primaryYButton.whenPressed(new ShiftHighGearCommand(driveTrainSubsystem));
      Buttons.primaryLB.whenPressed(new SetLowSpeedCommand(driveTrainSubsystem));
      Buttons.primaryRB.whenPressed(new SetNormalSpeedCommand(driveTrainSubsystem));

    // control panel
      Buttons.primaryXButton.whenPressed(new RotatePanelCommand(panelSubsystem));
      Buttons.primaryBButton.whenPressed(new AlignColorCommand(panelSubsystem));
    // secondary controls
      Buttons.secondaryDPadN.whenPressed(new RaiseIntakeArmCommand(intakeSubsystem));
      Buttons.secondaryDPadS.whenPressed(new LowerIntakeArmCommand(intakeSubsystem));    
      Buttons.secondaryBButton.whenHeld(new ReverseIndexToBlasterCommand(intakeSubsystem));
      Buttons.secondaryRB.whenHeld(new IndexToBlasterCommand(intakeSubsystem));
      //Buttons.secondaryAButton.whenHeld(new WinchCommand(hangSubsystem), false);
      //raiseElevatorCommand = new RaiseElevatorCommand(hangSubsystem, () -> Buttons.secondaryJoystick.getLeftY());    
      //gondolaCommand = new GondolaCommand(hangSubsystem, ()->Buttons.secondaryJoystick.getLeftX());
      
    // dashboard control buttons  
      //SmartDashboard.putData("trench 35 foot blaster velocity", new BlasterConstantOutputCommand(blasterSubsystem, RobotMap.LidarSensor, shooterVelocity));
      SmartDashboard.putData(new IndexToBlasterCommand(intakeSubsystem));

    // mode switching 
      //startIntakeCommand.addRequirements(elevatorSubsystem, conveyorSubsystem, alignmentBeltSubsystem);
      //secondaryBackButton.whenPressed(startIntakeCommand);
      //secondaryStartButton.whenPressed(new StopIntakeOuttakeCommand(intakeSubsystem))
  }

  private void dash(){
    //autoChooser.setDefaultOption("None", null);
    //autoChooser.addOption("Snag N' Yeet", new SnagNYeetCommandGroup(driveTrainSubsystem, intakeSubsystem, conveyorSubsystem, turretSubsystem, RobotMap.LidarSensor, blasterSubsystem, Buttons.secondaryJoystick));
    //autoChooser.setDefaultOption("Yeet3PushNom3", new Yeet3PushNom3(driveTrainSubsystem, intakeSubsystem, turretSubsystem, blasterSubsystem, RobotMap.LidarSensor, conveyorSubsystem));
    //autoChooser.addOption("Yeet3PushNom3", new Yeet3PushNom3(driveTrainSubsystem, intakeSubsystem, turretSubsystem, blasterSubsystem, RobotMap.LidarSensor, conveyorSubsystem, Buttons.secondaryJoystick));
    //autoChooser.addOption("Yeet3Nom3Yeet3", new Yeet3Nom3Yeet3(driveTrainSubsystem, intakeSubsystem, turretSubsystem, blasterSubsystem, RobotMap.LidarSensor, conveyorSubsystem, Buttons.secondaryJoystick));
    //autoChooser.addOption("PushNom2Yeet5Nom1", new PushNom2Yeet5Nom1(driveTrainSubsystem, intakeSubsystem, turretSubsystem, blasterSubsystem, RobotMap.LidarSensor, conveyorSubsystem, Buttons.secondaryJoystick));
    //autoChooser.setDefaultOption("yeet3", new Yeet3(turretSubsystem, driveTrainSubsystem, conveyorSubsystem, intakeSubsystem, RobotMap.LidarSensor, blasterSubsystem, Buttons.secondaryJoystick));
    //autoChooser.addOption("noturn", new NoTurnAuton(turretSubsystem, driveTrainSubsystem, conveyorSubsystem, intakeSubsystem, RobotMap.LidarSensor, blasterSubsystem, Buttons.secondaryJoystick));
    //autoChooser.addOption("NERDSAUTO", new NerdsAuton(turretSubsystem, driveTrainSubsystem, conveyorSubsystem, intakeSubsystem, RobotMap.LidarSensor, blasterSubsystem));
    //autoChooser.addOption("CrossBaseline", new CrossBaseline(driveTrainSubsystem));
    //autoChooser.addOption("Yeet3FinalsAuton", new Yeet3FinalsAuton(turretSubsystem, driveTrainSubsystem, conveyorSubsystem, intakeSubsystem, RobotMap.LidarSensor, blasterSubsystem, Buttons.secondaryJoystick));
    //autoChooser.addOption("90Degrees", autonTurn90DegreeCommand);
    //autoChooser.addOption("yeet3New", new Yeet3New(turretSubsystem, driveTrainSubsystem, conveyorSubsystem, intakeSubsystem, RobotMap.LidarSensor, blasterSubsystem, Buttons.secondaryJoystick));
    //autoChooser.addOption("0 to 45 to 0", new );
    //autoChooser.addOption("Nom2Yeet5", new Nom2Yeet5(driveTrainSubsystem, intakeSubsystem, turretSubsystem, blasterSubsystem, RobotMap.LidarSensor, conveyorSubsystem, Buttons.secondaryJoystick));
    //autoChooser.addOption("Nom2Turn45Yeet5", new Nom2Turn45Yeet5(driveTrainSubsystem, intakeSubsystem, turretSubsystem, blasterSubsystem, RobotMap.LidarSensor, conveyorSubsystem, Buttons.secondaryJoystick));
    //dashboard control buttons  
    //SmartDashboard.putData("10 foot blaster velocity", new BlasterConstantOutputCommand(blasterSubsystem, RobotMap.LidarSensor, Constants.AUTON_TARGET_CENTER_LINE_CONSTANT_VELOCITY));
    //SmartDashboard.putData("trench 35 foot blaster velocity", new BlasterConstantOutputCommand(blasterSubsystem, RobotMap.LidarSensor, Constants.TRENCH_SHOOTER_VELOCITY));
   
    SmartDashboard.putData(new IndexToBlasterCommand(intakeSubsystem));
    SmartDashboard.putData("Auton Mode", autoChooser);
  }

  private void setupDefaultCommands(){
    driveTrainSubsystem.setDefaultCommand(
        new DriveCommand(driveTrainSubsystem, 
        () -> deaden(Buttons.primaryJoystick.getLeftY()),
        () -> Buttons.primaryJoystick.getRightX())
        );   

    intakeSubsystem.setDefaultCommand(
        new StartIntakeCommand(intakeSubsystem, 
        () -> deaden(Buttons.secondaryJoystick.getRightY()))
        );
        
    conveyorSubsystem.setDefaultCommand(
        new ConveyorCommand(conveyorSubsystem, 
        ()-> deaden(Buttons.secondaryJoystick.getRightY()))
        );

    // hangSubsystem.setDefaultCommand(
    //     new RaiseElevatorCommand(hangSubsystem, 
    //     () -> deaden(Buttons.secondaryJoystick.getLeftY()), Buttons.secondaryStartButton.get())
    //     );
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
