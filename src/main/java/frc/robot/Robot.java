/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.autonCommands.TurnCommand;
import frc.robot.sensors.Gyro;
import frc.robot.subsystems.*;
import frc.robot.vision.GripPipeline;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private VisionProcessorSubsystem vision;
  private Thread visionThread;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    
    if (Robot.isReal()) {
      // Starts vision thread only if not running in simulation mode
      // Vision System calculates the angle to the target and posts it to the NetworkTable
      vision = new VisionProcessorSubsystem(RobotMap.RingLight, new GripPipeline());
      visionThread = vision.getVisionThread();
      visionThread.setDaemon(true);
      visionThread.start();
    }

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
    if (Robot.isReal()) {
      SmartDashboard.putNumber("ANGLE", vision.getAngle());
    }
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    RobotMap.FrontLeftMotor.setNeutralMode(NeutralMode.Coast);
    RobotMap.BackLeftMotor.setNeutralMode(NeutralMode.Coast);
    RobotMap.FrontRightMotor.setNeutralMode(NeutralMode.Coast);
    RobotMap.BackRightMotor.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    SmartDashboard.putString("auton selected", m_autonomousCommand.toString());

    
    System.out.println("Init Auton.........");
    Gyro.getInstance().reset();
    Gyro.getInstance().calibrationCheck(); // may take up to two seconds to complete
    System.out.println("Gyro Yaw at Startup: " + Gyro.getInstance().getYaw());
    // CommandScheduler.getInstance().cancelAll(); // cancel all teleop commands

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
    RobotMap.FrontLeftMotor.setNeutralMode(NeutralMode.Brake);
    RobotMap.BackLeftMotor.setNeutralMode(NeutralMode.Brake);
    RobotMap.FrontRightMotor.setNeutralMode(NeutralMode.Brake);
    RobotMap.BackRightMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // SmartDashboard.putNumber("yaw",gyroSubsystem.getYaw());

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    RobotMap.FrontLeftMotor.setNeutralMode(NeutralMode.Coast);
    RobotMap.BackLeftMotor.setNeutralMode(NeutralMode.Coast);
    RobotMap.FrontRightMotor.setNeutralMode(NeutralMode.Coast);
    RobotMap.BackRightMotor.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    Gyro.getInstance().reset();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      System.out.println("Scheduling test command");
      m_autonomousCommand.schedule();
    }
    
    
    RobotMap.FrontLeftMotor.setNeutralMode(NeutralMode.Brake);
    RobotMap.BackLeftMotor.setNeutralMode(NeutralMode.Brake);
    RobotMap.FrontRightMotor.setNeutralMode(NeutralMode.Brake);
    RobotMap.BackRightMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    
    // System.out.println("Running test periodic");
    // SmartDashboard.putNumber("pitch",gyroSubsystem.getPitch());
    // SmartDashboard.putNumber("roll",gyroSubsystem.getRoll());
    // SmartDashboard.putNumber("yaw",gyroSubsystem.getYaw());
    CommandScheduler.getInstance().run();

  }
}
