/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.sensors.ColorSensor;
import frc.robot.sensors.Gyro;
import frc.robot.sensors.Lidar;
import frc.robot.sensors.PhotoEye;


/**
 * Add your docs here.
 */
public class RobotMap {

    public static final WPI_TalonFX BlasterMotor = new WPI_TalonFX(Constants.BLASTER_MOTOR_PORT);
    public static final Solenoid BlasterHood = new Solenoid(Constants.RAISE_BLASTER_HOOD_SOL_PORT);
    
    public static final Gyro GyroSensor = Gyro.getInstance();
    public static final Lidar LidarSensor = Lidar.getInstance();

    public static final Solenoid GearShifter = new Solenoid(Constants.HIGH_GEAR_SOL_PORT);
	public static final WPI_TalonFX FrontRightMotor = new WPI_TalonFX(Constants.FR_TALON);
	public static final WPI_TalonFX FrontLeftMotor = new WPI_TalonFX(Constants.FL_TALON);
	public static final WPI_TalonFX BackLeftMotor = new WPI_TalonFX(Constants.BL_TALON);
	public static final BaseMotorController PanelMotor = new WPI_TalonSRX(Constants.PANEL_MOTOR_PORT);;
	public static final ColorSensor ColorSensor = new ColorSensor();
    public static WPI_TalonFX BackRightMotor = new WPI_TalonFX(Constants.BR_TALON);

    public static final WPI_VictorSPX ConveyorMotor = new WPI_VictorSPX(Constants.INFEED_CONVEYOR_MOTOR_PORT);
    public static final WPI_VictorSPX AlignmentBeltMotor = new WPI_VictorSPX(Constants.INFEED_CONVEYOR_INDEXER_MOTOR_PORT);

    public static final WPI_VictorSPX GondolaMotor = new WPI_VictorSPX(Constants.CLIMBING_GONDOLA_ADJUSTMENT_MOTOR_PORT);

    public static final WPI_VictorSPX HangMotor = new WPI_VictorSPX(Constants.CLIMBING_RAISE_ELEVATOR_MOTOR_PORT);
    public static final WPI_VictorSPX WinchMotor1 = new WPI_VictorSPX(Constants.CLIMBING_1_MOTOR_PORT);
    public static final WPI_VictorSPX WinchMotor2 = new WPI_VictorSPX(Constants.CLIMBING_2_MOTOR_PORT);

    public static final DigitalInput LimitSwitch1 = new DigitalInput(Constants.ELEVATOR_LIMIT_SWITCH_1_PORT);
    public static final DigitalInput LimitSwitch2 = new DigitalInput(Constants.ELEVATOR_LIMIT_SWITCH_2_PORT);

    public static final DoubleSolenoid ArmMover = new DoubleSolenoid(Constants.RAISE_POWER_CELL_INTAKE_SOL_PORT, Constants.LOWER_POWER_CELL_INTAKE_SOL_PORT); // raise = kforward lower = kreverse
    public static final WPI_VictorSPX IntakeMotor = new WPI_VictorSPX(Constants.INTAKE_MOTOR_PORT);
    public static final WPI_VictorSPX FeedToBlasterMotor = new WPI_VictorSPX(Constants.FEED_TO_BLASTER_MOTOR_PORT);
    
    public static final VictorSPX TurretMotor = new VictorSPX(Constants.TURRET_MOTOR_PORT);
	public static final DigitalInput LeftLimitSwitch = new DigitalInput(Constants.TURRET_LEFT_DIO);
	public static final DigitalInput RightLimitSwitch = new DigitalInput(Constants.TURRET_RIGHT_DIO);

    public static final Solenoid RingLight = new Solenoid(Constants.RING_LIGHT_PORT);

    public static final PhotoEye IntakePhotoEye = new PhotoEye(7);
	public static final PhotoEye SpacingPhotoEye = new PhotoEye(8);
    public static final PhotoEye ExitPhotoEye = new PhotoEye(9);

    public static final Counter IntakeCounter = new Counter(IntakePhotoEye.getDigitalInput());
    public static final Counter ExitCounter = new Counter(ExitPhotoEye.getDigitalInput());
}
