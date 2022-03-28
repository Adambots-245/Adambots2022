
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Gamepad.Buttons;
import frc.robot.Gamepad.GamepadConstants;

import frc.robot.commands.*;
import frc.robot.commands.autonCommands.*;
import frc.robot.commands.autonCommands.autonCommandGroups.Auton1Ball;
import frc.robot.commands.autonCommands.autonCommandGroups.Auton2Ball;
import frc.robot.commands.autonCommands.autonCommandGroups.Position1Auton3BallBlue;
import frc.robot.commands.autonCommands.autonCommandGroups.Position1Auton3BallRed;
import frc.robot.commands.autonCommands.autonCommandGroups.Position1Auton5Ball;
import frc.robot.commands.autonCommands.autonCommandGroups.Position2Auton4Ball;
import frc.robot.subsystems.*;
//import frc.robot.utils.Log;
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
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(RobotMap.GyroSensor, 
                                                                                  RobotMap.GearShifter, 
                                                                                  RobotMap.FrontRightMotor, 
                                                                                  RobotMap.FrontLeftMotor, 
                                                                                  RobotMap.BackLeftMotor, 
                                                                                  RobotMap.BackRightMotor);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(RobotMap.IntakeMotor,
                                                                        RobotMap.intakeExtend,
                                                                        RobotMap.intakePhotoEye,
                                                                        RobotMap.intakeCatapultPhotoEye,
                                                                        RobotMap.chooChooOpticalSensor
                                                                      );
  private final CatapultSubsystem catapultSubsystem = new CatapultSubsystem(RobotMap.ChooChooMotor, 
                                                                            RobotMap.chooChooOpticalSensor, 
                                                                            RobotMap.bandHomeSwitch,
                                                                            RobotMap.BandMotor
                                                                            );
  private final HangSubsystem hangSubsystem = new HangSubsystem(RobotMap.winchMotor1, 
                                                                RobotMap.winchMotor2, 
                                                                RobotMap.leftClampSwitch, 
                                                                RobotMap.rightClampSwitch, 
                                                                RobotMap.rungArmRetractedSwitch, 
                                                                RobotMap.rungArmMidSwitch, 
                                                                RobotMap.rungArmAdvancedSwitch, 
                                                                RobotMap.hangAngle, 
                                                                RobotMap.hangClamp,
                                                                RobotMap.leftRungSwitch,
                                                                RobotMap.rightRungSwitch);

  private final DebugSubsystem debugSubsystem = new DebugSubsystem();
  
  // commands
  private DriveForwardDistanceCommand autonDriveForwardDistanceCommand;
  private TurnToAngleCommand autonTurn90DegreeCommand;
  private GyroDriveForDistCommand autonGyroDriveForwardDistanceCommand;
  // private SequentialCommandGroup autonDriveForwardGyroDistanceCommand;
  
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

    // Set up button on smartdashboard to test commands
    SmartDashboard.putData("Turn -45 Deg.", new TurnToAngleCommand(driveTrainSubsystem, -45, true));
    SmartDashboard.putData("Turn 45 Deg.", new TurnToAngleCommand(driveTrainSubsystem, 45, true));
    SmartDashboard.putData("DriveForward",new  DriveForwardDistanceCommand(driveTrainSubsystem, Constants.ENCODER_TICKS_PER_INCH * 75, -0.75));

    SmartDashboard.putData("Prime Catapult", new CatapultPrimeCommand(catapultSubsystem));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
      // Primary Controls
      Buttons.primaryAButton.whenPressed(new CatapultTimeFireCommand(catapultSubsystem));
      Buttons.primaryRB.whenPressed(new BandHomeCommand(catapultSubsystem, Constants.HOME_TENSION));
      Buttons.secondaryDPadN.whileHeld(new RunBandCommand(catapultSubsystem, -1.0));
      Buttons.secondaryDPadS.whileHeld(new RunBandCommand(catapultSubsystem, 1.0));
      Buttons.primaryXButton.whenPressed(new BandMoveCommand(catapultSubsystem, Constants.TARMAC_TENSION));
      Buttons.primaryBButton.whenPressed(new BandMoveCommand(catapultSubsystem, Constants.SAFE_ZONE_TENSION));
      // Buttons.primaryLB.whenPressed(new CatapultBackdriveCommand(catapultSubsystem));
      // Buttons.primaryBButton.whenPressed(new CatapultStopInCommand(catapultSubsystem));
      // Buttons.primaryXButton.whenPressed(new CatapultStopOutCommand(catapultSubsystem));

      // Secondary Controls
      Buttons.secondaryRB.whileHeld(new WinchCommand(hangSubsystem));
      Buttons.secondaryLB.whileHeld(new UnwinchCommand(hangSubsystem));
      Buttons.secondaryAButton.whenPressed(new ClampRungCommand(hangSubsystem));
      Buttons.secondaryYButton.whenPressed(new UnclampRungCommand(hangSubsystem));
      Buttons.secondaryBButton.whenPressed(new MoveHangOutCommand(hangSubsystem));
      Buttons.secondaryXButton.whenPressed(new MoveHangInCommand(hangSubsystem));
      Buttons.secondaryBackButton.whenPressed(new IntakeOutCommand(intakeSubsystem));
      Buttons.secondaryStartButton.whenPressed(new IntakeInCommand(intakeSubsystem));

      // buttons for vision
      Buttons.primaryYButton.whenPressed(new AllignToHubCommand(driveTrainSubsystem));
      //Buttons.primaryYButton.whenPressed(new TurnToAngleCommand(driveTrainSubsystem, 0.1, 10, true));
  }

  private void dash(){
    autoChooser.setDefaultOption("None", null);
    autoChooser.addOption("Auton1Ball", new Auton1Ball(catapultSubsystem, driveTrainSubsystem, intakeSubsystem));
    autoChooser.addOption("Auton2Ball", new Auton2Ball(driveTrainSubsystem, intakeSubsystem, catapultSubsystem));
    autoChooser.addOption("RedPosition1Auton3Ball", new Position1Auton3BallRed(driveTrainSubsystem, intakeSubsystem, catapultSubsystem));
    autoChooser.addOption("BluePosition1Auton3Ball", new Position1Auton3BallBlue(driveTrainSubsystem, intakeSubsystem, catapultSubsystem));
    // autoChooser.addOption("Position1Auton5Ball", new Position1Auton5Ball(driveTrainSubsystem, intakeSubsystem, catapultSubsystem));
    // autoChooser.addOption("Position2Auton4Ball", new Position2Auton4Ball(driveTrainSubsystem, intakeSubsystem, catapultSubsystem));
   
    SmartDashboard.putData("Auton Mode", autoChooser);
  }

  private void setupDefaultCommands(){
    driveTrainSubsystem.setDefaultCommand(
       new DriveCommand(driveTrainSubsystem, 
       () -> deaden(-Buttons.primaryJoystick.getLeftY()),
        () -> Buttons.primaryJoystick.getRightX())
        );  

    intakeSubsystem.setDefaultCommand(
        new StartIntakeCommand(intakeSubsystem, 
        () -> deaden(Buttons.secondaryJoystick.getRightY()))
        );
    
    //  catapultSubsystem.setDefaultCommand(
    //    new RunBandCommand(catapultSubsystem, 1),
    //    () -> deaden(Buttons.secondaryJoystick.getLeftY())
    //  );
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
     if (autoChooser.getSelected() != null){
      Log.info("Chosen Auton Command: ", autoChooser.getSelected().toString());
    } else{
      Log.info("Chosen Auton Command: None");
      return new Auton2Ball(driveTrainSubsystem, intakeSubsystem, catapultSubsystem);
     }
     
      
    // return new Auton2Ball(driveTrainSubsystem, intakeSubsystem, catapultSubsystem);

     //System.out.println(autoChooser.getSelected().toString());
     return autoChooser.getSelected();
  }
}
