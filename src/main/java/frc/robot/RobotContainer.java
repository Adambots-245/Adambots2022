
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
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
import frc.robot.commands.autonCommands.autonCommandGroups.BarrelPathAuton;
import frc.robot.commands.autonCommands.autonCommandGroups.CrossBaseline;
import frc.robot.commands.autonCommands.autonCommandGroups.NerdsAuton;
import frc.robot.commands.autonCommands.autonCommandGroups.NoTurnAuton;
import frc.robot.commands.autonCommands.autonCommandGroups.Nom2Turn45Yeet5;
import frc.robot.commands.autonCommands.autonCommandGroups.Nom2Yeet5;
import frc.robot.commands.autonCommands.autonCommandGroups.PushNom2Yeet5;
import frc.robot.commands.autonCommands.autonCommandGroups.PushNom2Yeet5Nom1;
import frc.robot.commands.autonCommands.autonCommandGroups.SlalomPathAuton;
import frc.robot.commands.autonCommands.autonCommandGroups.SnagNYeetCommandGroup;
import frc.robot.commands.autonCommands.autonCommandGroups.Yeet3;
import frc.robot.commands.autonCommands.autonCommandGroups.Yeet3FinalsAuton;
import frc.robot.commands.autonCommands.autonCommandGroups.Yeet3New;
import frc.robot.commands.autonCommands.autonCommandGroups.Yeet3Nom3Yeet3;
import frc.robot.commands.autonCommands.autonCommandGroups.Yeet3PushNom3;
import frc.robot.subsystems.*;
import frc.robot.utils.Log;


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
  private final BlasterSubsystem blasterSubsystem = new BlasterSubsystem(RobotMap.BlasterMotor, RobotMap.BlasterHood);
  private final ControlPanelSubsystem panelSubsystem = new ControlPanelSubsystem(RobotMap.PanelMotor, RobotMap.ColorSensor);
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem(RobotMap.ConveyorMotor, RobotMap.AlignmentBeltMotor, RobotMap.IntakePhotoEye, RobotMap.SpacingPhotoEye, RobotMap.ExitPhotoEye);
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(RobotMap.GyroSensor, RobotMap.GearShifter, RobotMap.FrontRightMotor, RobotMap.FrontLeftMotor, RobotMap.BackLeftMotor, RobotMap.BackRightMotor);
  private final GondolaSubsystem gondolaSubsystem = new GondolaSubsystem(RobotMap.GondolaMotor);
  private final HangSubsystem hangSubsystem = new HangSubsystem(RobotMap.HangMotor, RobotMap.WinchMotor1, RobotMap.WinchMotor2, RobotMap.LimitSwitch1, RobotMap.LimitSwitch2);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(RobotMap.ArmMover, RobotMap.IntakeMotor, RobotMap.FeedToBlasterMotor);
  private TurretSubsystem turretSubsystem = new TurretSubsystem(RobotMap.TurretMotor, RobotMap.LeftLimitSwitch, RobotMap.RightLimitSwitch);
  
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

    //auton commands
    autonDriveForwardDistanceCommand = new DriveForwardDistanceCommand(driveTrainSubsystem,
        Constants.AUTON_DRIVE_FORWARD_DISTANCE, Constants.AUTON_DRIVE_FORWARD_SPEED);

    // autonGyroDriveForwardDistanceCommand = new GyroDriveForDistCommand(driveTrainSubsystem,
        // Constants.AUTON_DRIVE_FORWARD_DISTANCE, Constants.AUTON_DRIVE_FORWARD_SPEED, gyroSubsystem.getYaw());
        double autonSpeed = .75;
    autonDriveForwardGyroDistanceCommand = new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.AUTON_PUSH_ROBOT_DISTANCE, autonSpeed*.5, 0, true).andThen(new WaitCommand(1)).andThen(new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.AUTON_FORWARD_BALL_PICKUP_DISTANCE, -autonSpeed, 0, false));
    
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
      Buttons.primaryYButton.whenPressed(new ShiftHighGearCommand(driveTrainSubsystem));
      Buttons.primaryLB.whenPressed(new SetLowSpeedCommand(driveTrainSubsystem));
      Buttons.primaryRB.whenPressed(new SetNormalSpeedCommand(driveTrainSubsystem));

      //control panel
      Buttons.primaryXButton.whenPressed(new RotatePanelCommand(panelSubsystem));
      Buttons.primaryBButton.whenPressed(new AlignColorCommand(panelSubsystem));
      // Buttons.secondaryXButton.whenHeld(new PanelMotor(panelSubsystem)); //CHANGE THIS TO PRIMARY SOMEHOW

      // secondary controls
      // intake 
      
      Buttons.secondaryDPadN.whenPressed(new RaiseIntakeArmCommand(intakeSubsystem));
      Buttons.secondaryDPadS.whenPressed(new LowerIntakeArmCommand(intakeSubsystem));    
      // Buttons.secondaryYButton.whenHeld(new IndexToBlasterCommand(intakeSubsystem));  
      Buttons.secondaryBButton.whenHeld(new ReverseIndexToBlasterCommand(intakeSubsystem));
      Buttons.secondaryRB.whenHeld(new IndexToBlasterCommand(intakeSubsystem));  
      
      // Buttons.secondaryYButton.whenHeld(new IndexToBlasterCommand(intakeSubsystem));  
      
      // turret 
     
      Buttons.secondaryDPadN.whenPressed(new RaiseIntakeArmCommand(intakeSubsystem));
      Buttons.secondaryDPadS.whenPressed(new LowerIntakeArmCommand(intakeSubsystem));    
      Buttons.secondaryXButton.whileHeld(new TurnToTargetCommand(turretSubsystem, RobotMap.LidarSensor), false);
      // turretSubsystem.setDefaultCommand(new TurretManualCommand(turretSubsystem, ()->Buttons.secondaryJoystick.getTriggerAxis(Hand.kLeft), ()->Buttons.secondaryJoystick.getTriggerAxis(Hand.kRight)));
      
      // lidar susbsystem
        // Buttons.primaryXButton.whenPressed(new MeasureDistanceCommand(RobotMap.LidarSensor));
      
      // blaster  
      // Buttons.secondaryLB.toggleWhenPressed(new BlasterConstantOutputCommand(blasterSubsystem, RobotMap.LidarSensor));
      Buttons.secondaryLB.toggleWhenPressed(new BlasterDistanceBasedCommand(blasterSubsystem, RobotMap.LidarSensor, Buttons.secondaryJoystick));
      Buttons.secondaryYButton.whenReleased(new BackboardToggleCommand(blasterSubsystem));
      // blasterSubsystem.setDefaultCommand(new BlasterPercentOutput(blasterSubsystem, () -> Buttons.primaryJoystick.getTriggerAxis(Hand.kRight)));
      // hang 
      
      Buttons.secondaryAButton.whenHeld(new WinchCommand(hangSubsystem), false);
      //raiseElevatorCommand = new RaiseElevatorCommand(hangSubsystem, () -> Buttons.secondaryJoystick.getY(Hand.kLeft));    
      //gondolaCommand = new GondolaCommand(hangSubsystem, ()->Buttons.secondaryJoystick.getX(Hand.kLeft));
      
    // dashboard control buttons  
      SmartDashboard.putData("10 foot blaster velocity", new BlasterConstantOutputCommand(blasterSubsystem, RobotMap.LidarSensor, Constants.AUTON_TARGET_CENTER_LINE_CONSTANT_VELOCITY));
      // SmartDashboard.putData("trench 35 foot blaster velocity", new BlasterConstantOutputCommand(blasterSubsystem, RobotMap.LidarSensor, shooterVelocity));
      SmartDashboard.putData("trench 35 foot blaster velocity", new BlasterConstantOutputCommand(blasterSubsystem, RobotMap.LidarSensor, Constants.TRENCH_SHOOTER_VELOCITY));
      SmartDashboard.putData(new IndexToBlasterCommand(intakeSubsystem));

    // mode switching 
      // startIntakeCommand.addRequirements(elevatorSubsystem, conveyorSubsystem, alignmentBeltSubsystem);
      //secondaryBackButton.whenPressed(startIntakeCommand);
      // secondaryStartButton.whenPressed(new StopIntakeOuttakeCommand(intakeSubsystem));

    // test stuff 
      // Buttons.primaryYButton.whenPressed(new StartOuttakeCommand(intakeSubsystem));
      // Buttons.primaryAButton.whenReleased(new StopIntakeOuttakeCommand(intakeSubsystem));
      // //Buttons.primaryAButton.whileHeld(new TestCo  mmand());
      // Buttons.primaryYButton.whenReleased(new StopIntakeOuttakeCommand(intakeSubsystem));

      // Turret subsystem
      //TurretManualCommand turretManualCommand = new TurretManualCommand(turretSubsystem,
      //    () -> Buttons.secondaryJoystick.getTriggerAxis(Hand.kLeft), () -> Buttons.secondaryJoystick.getTriggerAxis(Hand.kRight));
      //Buttons.secondaryLB.whenHeld(new TurnToTargetCommand(turretSubsystem));

  }

  private void dash(){
    // autoChooser.setDefaultOption("None", null);
    autoChooser.addOption("Snag N' Yeet", new SnagNYeetCommandGroup(driveTrainSubsystem, intakeSubsystem, conveyorSubsystem, turretSubsystem, RobotMap.LidarSensor, blasterSubsystem, Buttons.secondaryJoystick));
    // autoChooser.setDefaultOption("Yeet3PushNom3", new Yeet3PushNom3(driveTrainSubsystem, intakeSubsystem, turretSubsystem, blasterSubsystem, RobotMap.LidarSensor, conveyorSubsystem));
    // autoChooser.addOption("Yeet3PushNom3", new Yeet3PushNom3(driveTrainSubsystem, intakeSubsystem, turretSubsystem, blasterSubsystem, RobotMap.LidarSensor, conveyorSubsystem, Buttons.secondaryJoystick));
    autoChooser.addOption("Yeet3Nom3Yeet3", new Yeet3Nom3Yeet3(driveTrainSubsystem, intakeSubsystem, turretSubsystem, blasterSubsystem, RobotMap.LidarSensor, conveyorSubsystem, Buttons.secondaryJoystick));
    // autoChooser.addOption("PushNom2Yeet5Nom1", new PushNom2Yeet5Nom1(driveTrainSubsystem, intakeSubsystem, turretSubsystem, blasterSubsystem, RobotMap.LidarSensor, conveyorSubsystem, Buttons.secondaryJoystick));
    autoChooser.setDefaultOption("yeet3", new Yeet3(turretSubsystem, driveTrainSubsystem, conveyorSubsystem, intakeSubsystem, RobotMap.LidarSensor, blasterSubsystem, Buttons.secondaryJoystick));
    // autoChooser.addOption("noturn", new NoTurnAuton(turretSubsystem, driveTrainSubsystem, conveyorSubsystem, intakeSubsystem, RobotMap.LidarSensor, blasterSubsystem, Buttons.secondaryJoystick));
    // autoChooser.addOption("NERDSAUTO", new NerdsAuton(turretSubsystem, driveTrainSubsystem, conveyorSubsystem, intakeSubsystem, RobotMap.LidarSensor, blasterSubsystem));
    autoChooser.addOption("CrossBaseline", new CrossBaseline(driveTrainSubsystem));
    // autoChooser.addOption("Yeet3FinalsAuton", new Yeet3FinalsAuton(turretSubsystem, driveTrainSubsystem, conveyorSubsystem, intakeSubsystem, RobotMap.LidarSensor, blasterSubsystem, Buttons.secondaryJoystick));
    // autoChooser.addOption("90Degrees", autonTurn90DegreeCommand);
    autoChooser.addOption("yeet3New", new Yeet3New(turretSubsystem, driveTrainSubsystem, conveyorSubsystem, intakeSubsystem, RobotMap.LidarSensor, blasterSubsystem, Buttons.secondaryJoystick));
    // autoChooser.addOption("0 to 45 to 0", new );
    // autoChooser.addOption("Nom2Yeet5", new Nom2Yeet5(driveTrainSubsystem, intakeSubsystem, turretSubsystem, blasterSubsystem, RobotMap.LidarSensor, conveyorSubsystem, Buttons.secondaryJoystick));
    // autoChooser.addOption("Nom2Turn45Yeet5", new Nom2Turn45Yeet5(driveTrainSubsystem, intakeSubsystem, turretSubsystem, blasterSubsystem, RobotMap.LidarSensor, conveyorSubsystem, Buttons.secondaryJoystick));
    // dashboard control buttons  
    SmartDashboard.putData("10 foot blaster velocity", new BlasterConstantOutputCommand(blasterSubsystem, RobotMap.LidarSensor, Constants.AUTON_TARGET_CENTER_LINE_CONSTANT_VELOCITY));
    SmartDashboard.putData("trench 35 foot blaster velocity", new BlasterConstantOutputCommand(blasterSubsystem, RobotMap.LidarSensor, Constants.TRENCH_SHOOTER_VELOCITY));
   
    SmartDashboard.putData(new IndexToBlasterCommand(intakeSubsystem));

    SmartDashboard.putData("Auton Mode", autoChooser);
  }

  private void setupDefaultCommands(){
    driveTrainSubsystem.setDefaultCommand(
        new DriveCommand(driveTrainSubsystem, 
        () -> deaden(Buttons.primaryJoystick.getY(Hand.kLeft)),
        () -> Buttons.primaryJoystick.getX(Hand.kRight))
        );  

    intakeSubsystem.setDefaultCommand(
        new StartIntakeCommand(intakeSubsystem, 
        () -> deaden(Buttons.secondaryJoystick.getY(Hand.kRight)))
        );
        
    conveyorSubsystem.setDefaultCommand(
        new ConveyorCommand(conveyorSubsystem, 
        ()-> deaden(Buttons.secondaryJoystick.getY(Hand.kRight)))
        );

    hangSubsystem.setDefaultCommand(
        new RaiseElevatorCommand(hangSubsystem, 
        () -> deaden(Buttons.secondaryJoystick.getY(Hand.kLeft)), Buttons.secondaryStartButton.get())
        );
        
    gondolaSubsystem.setDefaultCommand(
        new GondolaCommand(gondolaSubsystem, 
        () -> deaden(Buttons.secondaryJoystick.getX(Hand.kLeft)))
        );
    
    turretSubsystem.setDefaultCommand(
        new ManualTurretCommand(turretSubsystem,
          ()->Math.pow(Buttons.secondaryJoystick.getTriggerAxis(Hand.kLeft), 2), 
          ()->Math.pow(deaden(Buttons.secondaryJoystick.getTriggerAxis(Hand.kRight)), 2))
        );
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
    // autonDriveForwardDistanceCommand will run in autonomous
    // return autonDriveForwardGyroDistanceCommand;
    // return new  DriveForwardGyroDistanceCommand(driveTrainSubsystem, 0, 0, 0, true).andThen(new DriveForwardGyroDistanceCommand(driveTrainSubsystem, 3500*48, -.75, 0, true)).andThen(new DriveForwardGyroDistanceCommand(driveTrainSubsystem, 3500*84, -.5, 90, false));
    // return autonTurn90DegreeCommand.andThen(new WaitCommand(3)).andThen(new TurnToAngleCommand(driveTrainSubsystem, 0.5, -45, false));
    System.out.println(autoChooser.getSelected());
    return autoChooser.getSelected();
    // return new LowerIntakeArmCommand(intakeSubsystem)
    // .andThen(new WaitCommand(4))
    // .andThen(new TurnToAngleFromCameraCommand(driveTrainSubsystem))
    // .andThen(new DriveToBallCommand(driveTrainSubsystem, intakeSubsystem, conveyorSubsystem, RobotMap.IntakePhotoEye));
    // .andThen(new StartIntakeCommand(intakeSubsystem, () -> -1.0))
    // .andThen(new DriveForwardDistanceCommand(driveTrainSubsystem, 20000, -Constants.AUTON_DRIVE_FORWARD_SPEED))
  }
}
