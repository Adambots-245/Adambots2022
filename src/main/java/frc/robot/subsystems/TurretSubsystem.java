/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.utils.Log;

public class TurretSubsystem extends PIDSubsystem {

  private VictorSPX turretMotor; //= new VictorSPX(Constants.TURRET_MOTOR_PORT); 
  private DigitalInput leftLimitSwitch; // = new DigitalInput(Constants.TURRET_LEFT_DIO);
  private DigitalInput rightLimitSwitch; // = new DigitalInput(Constants.TURRET_RIGHT_DIO);
  
  
  private NetworkTable table;
  private double angleOffset = 5;
  //private final SimpleMotorFeedforward m_shooterFeedforward =
  //    new SimpleMotorFeedforward(ShooterConstants.kSVolts,
  //                               ShooterConstants.kVVoltSecondsPerRotation);

  /**
   * The shooter subsystem for the robot.
   */
  public TurretSubsystem(VictorSPX turretMotor, DigitalInput leftLimitSwitch, DigitalInput rightLimitSwitch) {
    super(new PIDController(Constants.TURRET_kP, Constants.TURRET_kI, Constants.TURRET_kD));
    getController().setTolerance(Constants.TURRET_TOLERANCE);
    setSetpoint(Constants.TURRET_TARGET_ANGLE + angleOffset);
    // getController().setIntegratorRange(-0.05, 0.05);
    
    this.turretMotor = turretMotor;
    this.leftLimitSwitch = leftLimitSwitch;
    this.rightLimitSwitch = rightLimitSwitch;

    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    table = instance.getTable("Vision");

    //Log.infoF("Initializing TurretSubsystem - kP=%f, kI=%f, kD=%f", Constants.TURRET_kP, Constants.TURRET_kI, Constants.TURRET_kD);
  }

  public void setAngleOffset(double calculatedOffset) {
    angleOffset = calculatedOffset;
    SmartDashboard.putNumber("angleOffset", angleOffset);

    //Log.infoF("Setting Angle Offset to %f", angleOffset);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    System.out.println("Output: " + output);
    System.out.println("Current Angle in Output: " + table.getEntry("Angle").getDouble(0));
    //Log.info("Setpoint in useOutput: ", setpoint);

    Double measurement = getMeasurement();
    double calculatedOutput = getController().calculate(measurement, setpoint);
    SmartDashboard.putNumber("Calculated Output", calculatedOutput);

    // System.out.println("Calculated Output: " + calculatedOutput);
    double clampAbs = MathUtil.clamp(Math.abs(calculatedOutput), 0.1, 1.0);
    clampAbs = Math.signum(calculatedOutput) * clampAbs;

    SmartDashboard.putNumber("Turret Output", clampAbs);
    // SmartDashboard.putNumber("Turret Output", calculatedOutput);
    setSpeed(-calculatedOutput);  

    //Log.infoF("Use Output - PIDOutput=%f, Setpoint=%f, Measurement=%f, PIDCalculatedOutput=%f, Speed=%f", output, setpoint, measurement, calculatedOutput, -calculatedOutput);
    // setSpeed(-clampAbs);  
  }

  @Override
  public double getMeasurement() {
    // System.out.println("Current Angle: " + table.getEntry("Angle").getDouble(0));
    return table.getEntry("Angle").getDouble(0);
  }

  public boolean atSetpoint() {
    //Log.infoF("At Set Point: %b", m_controller.atSetpoint());
    return m_controller.atSetpoint();
  }

  public void runTurret(double speed) {

    //Log.infoF("Running turret at Speed=%f", speed);
    setSpeed(speed);
  }

  public void stopTurret() {
    //Log.info("Stopping Turret");
    setSpeed(Constants.STOP_MOTOR_SPEED);
  }

  public double getOffset() {
    return angleOffset;
  }

  public void changeOffset(double value) {
    angleOffset += value;

    //Log.infoF("Changing angle offset = %f", angleOffset);
    //Log.infoF("Changing setpoint = %f", Constants.TURRET_TARGET_ANGLE + angleOffset);
    setSetpoint(Constants.TURRET_TARGET_ANGLE + angleOffset);
  }

  public void setSpeed(double speed){
    if (!leftLimitSwitch.get()) {
      if (speed < 0){
        //Log.infoF("Stopping motor - speed = %f", speed);

        turretMotor.set(ControlMode.PercentOutput, Constants.STOP_MOTOR_SPEED);
      }
      else{
       
        //Log.infoF("Setting speed = %f", speed);

        turretMotor.set(ControlMode.PercentOutput, speed);
      }
      
    } else if (!rightLimitSwitch.get()) {
      if (speed > 0){
        //Log.infoF("Stopping motor - speed = %f", speed);

        turretMotor.set(ControlMode.PercentOutput, Constants.STOP_MOTOR_SPEED);
      }
      else {
        //Log.infoF("Setting speed = %f", speed);
        
        turretMotor.set(ControlMode.PercentOutput, speed);
      }
    } else {
        //Log.infoF("Setting speed = %f", speed);
        
        turretMotor.set(ControlMode.PercentOutput, speed);
    }

    //Log.infoF("Set Speed: %f - %f", speed, turretMotor.getMotorOutputVoltage());
  }



}
