
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
  // private final BlasterSubsystem blasterSubsystem = new BlasterSubsystem(RobotMap.BlasterMotor, RobotMap.BlasterHood);
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(RobotMap.GyroSensor, RobotMap.GearShifter, RobotMap.FrontRightMotor, RobotMap.FrontLeftMotor, RobotMap.BackLeftMotor, RobotMap.BackRightMotor);
  // private final GondolaSubsystem gondolaSubsystem = new GondolaSubsystem(RobotMap.GondolaMotor);
  // private final HangSubsystem hangSubsystem = new HangSubsystem(RobotMap.HangMotor, RobotMap.WinchMotor1, RobotMap.WinchMotor2, RobotMap.LimitSwitch1, RobotMap.LimitSwitch2);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(RobotMap.IntakeMotor);
  private final CatapultSubsystem catapultSubsystem = new CatapultSubsystem(RobotMap.ChooChooMotor, RobotMap.ChooChooLimitSwitch, RobotMap.BandMotor, RobotMap.BandLimitSwitch);
  
  // commands
  private DriveForwardDistanceCommand autonDriveForwardDistanceCommand;
  private TurnToAngleCommand autonTurn90DegreeCommand;
  private GyroDriveForDistCommand autonGyroDriveForwardDistanceCommand;
  // private SequentialCommandGroup autonDriveForwardGyroDistanceCommand;
  // private WinchCommand winchCommand;
  
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
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
      // primary controls
      // Buttons.primaryAButton.whenPressed(new ShiftLowGearCommand(driveTrainSubsystem));
      // Buttons.primaryYButton.whenPressed(new ShiftHighGearCommand(driveTrainSubsystem));
      Buttons.primaryRB.whenPressed(new SetNormalSpeedCommand(driveTrainSubsystem));

      // Buttons.secondaryLB.toggleWhenPressed(new BlasterDistanceBasedCommand(blasterSubsystem, RobotMap.LidarSensor, Buttons.secondaryJoystick));

      Buttons.primaryAButton.whenPressed(new CatapultFireCommand(catapultSubsystem));
      Buttons.primaryDPadN.whenPressed(new BandMoveCommand(catapultSubsystem, 3000D));
      Buttons.primaryDPadS.whenPressed(new BandMoveCommand(catapultSubsystem, 1000D));

     // Buttons.secondaryBButton.whenPressed(new StartIntakeCommand(intakeSubsystem, () -> -1));

      //control panel
      // Buttons.primaryXButton.whenPressed(new RotatePanelCommand(panelSubsystem));
      // Buttons.primaryBButton.whenPressed(new AlignColorCommand(panelSubsystem));
      // Buttons.secondaryXButton.whenHeld(new PanelMotor(panelSubsystem)); //CHANGE THIS TO PRIMARY SOMEHOW

      // secondary controls
      // intake 
      
     // Buttons.secondaryDPadN.whenPressed(new RaiseIntakeArmCommand(intakeSubsystem));
      //Buttons.secondaryDPadS.whenPressed(new LowerIntakeArmCommand(intakeSubsystem));    
      // Buttons.secondaryYButton.whenHeld(new IndexToBlasterCommand(intakeSubsystem));  
     // Buttons.secondaryBButton.whenHeld(new ReverseIndexToBlasterCommand(intakeSubsystem));
     // Buttons.secondaryRB.whenHeld(new IndexToBlasterCommand(intakeSubsystem));  
      
      // Buttons.secondaryYButton.whenHeld(new IndexToBlasterCommand(intakeSubsystem));  
      
      // turret 
     
     // Buttons.secondaryDPadN.whenPressed(new RaiseIntakeArmCommand(intakeSubsystem));
     // Buttons.secondaryDPadS.whenPressed(new LowerIntakeArmCommand(intakeSubsystem));    
      // Buttons.secondaryXButton.whileHeld(new TurnToTargetCommand(turretSubsystem, RobotMap.LidarSensor), false);
      // turretSubsystem.setDefaultCommand(new TurretManualCommand(turretSubsystem, ()->Buttons.secondaryJoystick.getLeftTriggerAxis(), ()->Buttons.secondaryJoystick.getRightTriggerAxis()));
      
      // lidar susbsystem
        // Buttons.primaryXButton.whenPressed(new MeasureDistanceCommand(RobotMap.LidarSensor));
      
      // blaster  
      // Buttons.secondaryLB.toggleWhenPressed(new BlasterConstantOutputCommand(blasterSubsystem, RobotMap.LidarSensor));
      // Buttons.secondaryLB.toggleWhenPressed(new BlasterDistanceBasedCommand(blasterSubsystem, RobotMap.LidarSensor, Buttons.secondaryJoystick));
      // Buttons.secondaryYButton.whenReleased(new BackboardToggleCommand(blasterSubsystem));
      // blasterSubsystem.setDefaultCommand(new BlasterPercentOutput(blasterSubsystem, () -> Buttons.primaryJoystick.getRightTriggerAxis()));
      // hang 
      
      // Buttons.secondaryAButton.whenHeld(new WinchCommand(hangSubsystem), false);
      //raiseElevatorCommand = new RaiseElevatorCommand(hangSubsystem, () -> Buttons.secondaryJoystick.getLeftY());    
      //gondolaCommand = new GondolaCommand(hangSubsystem, ()->Buttons.secondaryJoystick.getLeftX());
      
    // dashboard control buttons  
      // SmartDashboard.putData("10 foot blaster velocity", new BlasterConstantOutputCommand(blasterSubsystem, RobotMap.LidarSensor, Constants.AUTON_TARGET_CENTER_LINE_CONSTANT_VELOCITY));
      // SmartDashboard.putData("trench 35 foot blaster velocity", new BlasterConstantOutputCommand(blasterSubsystem, RobotMap.LidarSensor, shooterVelocity));
      // SmartDashboard.putData("trench 35 foot blaster velocity", new BlasterConstantOutputCommand(blasterSubsystem, RobotMap.LidarSensor, Constants.TRENCH_SHOOTER_VELOCITY));
     // SmartDashboard.putData(new IndexToBlasterCommand(intakeSubsystem));

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
      //    () -> Buttons.secondaryJoystick.getLeftTriggerAxis(), () -> Buttons.secondaryJoystick.getRightTriggerAxis());
      //Buttons.secondaryLB.whenHeld(new TurnToTargetCommand(turretSubsystem));

  }

  private void dash(){
    // autoChooser.setDefaultOption("None", null);
   // autoChooser.addOption("Snag N' Yeet", new SnagNYeetCommandGroup(driveTrainSubsystem, intakeSubsystem, conveyorSubsystem, turretSubsystem, RobotMap.LidarSensor, blasterSubsystem, Buttons.secondaryJoystick));
    // autoChooser.setDefaultOption("Yeet3PushNom3", new Yeet3PushNom3(driveTrainSubsystem, intakeSubsystem, turretSubsystem, blasterSubsystem, RobotMap.LidarSensor, conveyorSubsystem));
   
  //`  SmartDashboard.putData(new IndexToBlasterCommand(intakeSubsystem));

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
    // return new LowerIntakeArmCommand(intakeSubsystem)
    // .andThen(new WaitCommand(4))
    // .andThen(new TurnToAngleFromCameraCommand(driveTrainSubsystem))
    // .andThen(new DriveToBallCommand(driveTrainSubsystem, intakeSubsystem, conveyorSubsystem, RobotMap.IntakePhotoEye));
    // .andThen(new StartIntakeCommand(intakeSubsystem, () -> -1.0))
    // .andThen(new DriveForwardDistanceCommand(driveTrainSubsystem, 20000, -Constants.AUTON_DRIVE_FORWARD_SPEED))
  }
}
