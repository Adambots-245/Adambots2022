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

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import frc.robot.sensors.ColorSensor;
import frc.robot.sensors.Gyro;
import frc.robot.sensors.Lidar;
import frc.robot.sensors.PhotoEye;


/**
 * Add your docs here.
 */
public class RobotMap {
    public static final Solenoid CatapultStop = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.RAISE_CATAPULT_STOP_SOL_PORT);

    // public static final WPI_TalonFX BlasterMotor = new WPI_TalonFX(Constants.BLASTER_MOTOR_PORT);
    // public static final Solenoid BlasterHood = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.RAISE_BLASTER_HOOD_SOL_PORT);
    public static final WPI_VictorSPX ChooChooMotor = new WPI_VictorSPX(Constants.CHOO_CHOO_MOTOR_PORT);
	public static final WPI_TalonSRX BandMotor = new WPI_TalonSRX(Constants.BAND_MOTOR_PORT);
    // public static final DoubleSolenoid RungClamp = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.RAISE_HANG_CLAMP_SOL_PORT, Constants.LOWER_HANG_CLAMP_SOL_PORT);

    // public static final WPI_TalonFX BlasterMotor = new WPI_TalonFX(Constants.BLASTER_MOTOR_PORT);
    // public static final Solenoid BlasterHood = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.RAISE_BLASTER_HOOD_SOL_PORT);
    
    public static final Gyro GyroSensor = Gyro.getInstance();
    public static final Accelerometer Accelerometer = new BuiltInAccelerometer();

    // public static final Lidar LidarSensor = Lidar.getInstance();

    public static final Solenoid GearShifter = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.HIGH_GEAR_SOL_PORT);
	public static final WPI_TalonFX FrontRightMotor = new WPI_TalonFX(Constants.FR_TALON);
	public static final WPI_TalonFX FrontLeftMotor = new WPI_TalonFX(Constants.FL_TALON);
    public static final WPI_TalonFX BackRightMotor = new WPI_TalonFX(Constants.BR_TALON);
	public static final WPI_TalonFX BackLeftMotor = new WPI_TalonFX(Constants.BL_TALON);
	// public static final BaseMotorController PanelMotor = new WPI_TalonSRX(Constants.PANEL_MOTOR_PORT);;
	public static final ColorSensor ColorSensor = new ColorSensor();

    // public static final WPI_VictorSPX ConveyorMotor = new WPI_VictorSPX(Constants.INFEED_CONVEYOR_MOTOR_PORT);
    // public static final WPI_VictorSPX AlignmentBeltMotor = new WPI_VictorSPX(Constants.INFEED_CONVEYOR_INDEXER_MOTOR_PORT);

    // public static final WPI_VictorSPX GondolaMotor = new WPI_VictorSPX(Constants.CLIMBING_GONDOLA_ADJUSTMENT_MOTOR_PORT);

    public static final WPI_VictorSPX winchMotor1 = new WPI_VictorSPX(Constants.HANG_WINCH_1_MOTOR_PORT);
    public static final WPI_VictorSPX winchMotor2 = new WPI_VictorSPX(Constants.HANG_WINCH_2_MOTOR_PORT);
    public static final DoubleSolenoid hangClamp = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.RAISE_HANG_CLAMP_SOL_PORT, Constants.LOWER_HANG_CLAMP_SOL_PORT);
    public static final DoubleSolenoid hangAngle = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.HANG_ARM_OUT_SOL_PORT, Constants.HANG_ARM_IN_SOL_PORT);

    public static final DigitalInput chooChooOpticalSensor = new DigitalInput(Constants.CHOO_CHOO_OPTICAL_SENSOR_PORT);
    public static final DigitalInput bandHomeSwitch = new DigitalInput(Constants.BAND_HOME_LIMIT_SWITCH_PORT);
    public static final DigitalInput leftClampSwitch = new DigitalInput(Constants.LEFT_CLAMP_LIMIT_SWITCH_PORT);
    public static final DigitalInput rightClampSwitch = new DigitalInput(Constants.RIGHT_CLAMP_LIMIT_SWITCH_PORT);
    public static final DigitalInput leftRungSwitch = new DigitalInput(Constants.LEFT_RUNG_LIMIT_SWITCH_PORT);
    public static final DigitalInput rightRungSwitch = new DigitalInput(Constants.RIGHT_RUNG_LIMIT_SWITCH_PORT);
    public static final PhotoEye rungArmAdvancedSwitch = new PhotoEye(Constants.RUNG_ARM_ADVANCED_PHOTO_EYE_PORT);
    public static final PhotoEye rungArmRetractedSwitch = new PhotoEye(Constants.RUNG_ARM_RETRACTED_PHOTO_EYE_PORT);
    public static final PhotoEye rungArmMidSwitch = new PhotoEye(Constants.RUNG_ARM_MID_PHOTO_EYE_PORT);
    public static final PhotoEye intakeBallDetector = new PhotoEye(Constants.BALL_DETECTOR_INTAKE_PORT);

    //public static final PhotoEye intakeBallSwitch = new PhotoEye(Constants.INTAKE_PHOTO_EYE_PORT);
    

    // public static final DoubleSolenoid ArmMover = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.RAISE_POWER_CELL_INTAKE_SOL_PORT, Constants.LOWER_POWER_CELL_INTAKE_SOL_PORT); // raise = kforward lower = kreverse
    public static final WPI_VictorSPX IntakeMotor = new WPI_VictorSPX(Constants.INTAKE_MOTOR_PORT);
    // public static final WPI_VictorSPX FeedToBlasterMotor = new WPI_VictorSPX(Constants.FEED_TO_BLASTER_MOTOR_PORT);
    
    // public static final VictorSPX TurretMotor = new VictorSPX(Constants.TURRET_MOTOR_PORT);
	// public static final DigitalInput LeftLimitSwitch = new DigitalInput(Constants.TURRET_LEFT_DIO);
	// public static final DigitalInput RightLimitSwitch = new DigitalInput(Constants.TURRET_RIGHT_DIO);

	//public static final DigitalInput ChooChooLimitSwitch = new DigitalInput(Constants.CHOO_CHOO_DIO);

    public static final Solenoid RingLight = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.RING_LIGHT_PORT);
    public static final Solenoid YellowLight = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.YELLOW_LEDS_SOL_PORT);


    // public static final PhotoEye IntakePhotoEye = new PhotoEye(7);
	// public static final PhotoEye SpacingPhotoEye = new PhotoEye(8);
    // public static final PhotoEye ExitPhotoEye = new PhotoEye(9);

    // public static final Counter IntakeCounter = new Counter(IntakePhotoEye.getDigitalInput());
    // public static final Counter ExitCounter = new Counter(ExitPhotoEye.getDigitalInput());
}
