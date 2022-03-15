/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.opencv.core.*;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorMatch;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */

public final class Constants {
    public static final double HOME_TENSION = 5.0;
    public static final double TARMAC_TENSION = 3.9;
    public static final double SAFE_ZONE_TENSION = 1.1;
    // motor IDs
    public static final int FL_TALON = 12;
    public static final int BL_TALON = 13;
    public static final int FR_TALON = 10;
    public static final int BR_TALON = 11;
    // public static final int BLASTER_MOTOR_PORT = 14;
    public static final int CHOO_CHOO_MOTOR_PORT = 24;
    public static final int BAND_MOTOR_PORT = 14;

    // public static final int HANG_ANGLE_1_MOTOR_PORT = 33;
    // public static final int HANG_ANGLE_2_MOTOR_PORT = 32;
    public static final int HANG_WINCH_1_MOTOR_PORT = 15;
    public static final int HANG_WINCH_2_MOTOR_PORT = 16;
    public static final int CLIMBING_RAISE_ELEVATOR_MOTOR_PORT = 23;
    public static final int CLIMBING_GONDOLA_ADJUSTMENT_MOTOR_PORT = 24;

    public static final int INTAKE_MOTOR_PORT = 18;
    // public static final int INFEED_CONVEYOR_MOTOR_PORT = 19;
    // public static final int INFEED_CONVEYOR_INDEXER_MOTOR_PORT = 20;
    // public static final int FEED_TO_BLASTER_MOTOR_PORT = 21;

    // solenoid IDs
    public static final int HIGH_GEAR_SOL_PORT = 7;

    //public static final int RAISE_POWER_CELL_INTAKE_SOL_PORT = 0;
    //public static final int LOWER_POWER_CELL_INTAKE_SOL_PORT = 1;

    public static final int LOWER_HANG_CLAMP_SOL_PORT = 0;
    public static final int RAISE_HANG_CLAMP_SOL_PORT = 1;
    public static final int HANG_ARM_OUT_SOL_PORT = 2;
    public static final int HANG_ARM_IN_SOL_PORT = 3;

    public static final int RAISE_CATAPULT_STOP_SOL_PORT = 5;
    // public static final int RAISE_BLASTER_HOOD_SOL_PORT = 2;

	public static final int RING_LIGHT_PORT = 4;
    public static final int YELLOW_LEDS_SOL_PORT = 6;

    // sensor IDs
    public static final int LIDAR_DIO = 5; //Not used

    // public static final int TURRET_LEFT_DIO = 4;
    // public static final int TURRET_RIGHT_DIO = 3;

    public static final int CHOO_CHOO_DIO = 9;
    
	// public static final int ELEVATOR_LIMIT_SWITCH_1_PORT = 0;
	// public static final int ELEVATOR_LIMIT_SWITCH_2_PORT = 1;
    public static final int RIGHT_RUNG_CLAMPED_LIMIT_SWITCH_PORT = 1;
    public static final int LEFT_RUNG_CLAMPED_LIMIT_SWITCH_PORT = 0;
    public static final int RIGHT_HANG_RUNG_IN_LIMIT_SWITCH_PORT = 5;
    public static final int BAND_HOME_LIMIT_SWITCH_PORT = 6;
    public static final int LEFT_HANG_RUNG_IN_LIMIT_SWITCH_PORT = 4;
    public static final int RUNG_ARM_ADVANCED_PHOTO_EYE_PORT = 2;
    public static final int RUNG_ARM_RETRACTED_PHOTO_EYE_PORT = 3;
    public static final int RUNG_ARM_MID_PHOTO_EYE_PORT = 8;

    // speed values
    public static final int INTAKE_SPEED = 1;
    public static final int OUTTAKE_SPEED = -1;
    public static final double STOP_MOTOR_SPEED = 0;
    public static final double NORMAL_SPEED_MODIFIER = 1;
    public static final double LOW_SPEED_MODIFIER = 0.5;
    public static final double MAX_MOTOR_SPEED = 1;
    public static final double WINCH_SPEED = -1;
    // public static final double CONVEYOR_SPEED = 0.5; // test speeds
    // public static final double CONVEYOR_INDEXER_SPEED = 0.5;
    // public static final double FEED_TO_BLASTER_SPEED = 0.5;
    public static final double MAX_BAND_MOVE_SPEED = 0.75; //test speeds

    public final static I2C.Port I2C_PORT = I2C.Port.kOnboard;

    public final static Color UNKNOWN_TARGET = Color.fromHSV(0, 0, 0);
    
    public final static Color BLUE_TARGET = Color.kBlue; //ColorMatch.makeColor(0.125, 0.424, 0.450);
    // public final static Color GREEN_TARGET = Color.kGreen; // ColorMatch.makeColor(0.167, 0.580, 0.252);
    public final static Color RED_TARGET = Color.kRed; //ColorMatch.makeColor(0.518, 0.347, 0.134);
    // public final static Color YELLOW_TARGET = Color.kYellow; // ColorMatch.makeColor(0.311, 0.566, 0.121);

    // Note: The direction that the color wheel itself turns will be the opposite of
    // the below direction
    // public enum DIRECTIONS {
        // CLOCKWISE,
        // COUNTERCLOCKWISE
    // }

    // public final static DIRECTIONS SPIN_DIRECTION = DIRECTIONS.COUNTERCLOCKWISE;

    // The distance between our color sensor and the game's color sensor in number
    // of color slices away
    // public final static int DIFFERENTIAL = 2;

    // PID slots
    public static final int DRIVE_PID_SLOT = 0;

    // Acceleration ramping constant for drive train
    public static final double SEC_NEUTRAL_TO_FULL = 0.4;

    // Vision Constants
    public static final int BALL_CAM_NUMBER = 1;
    public static final int HUB_CAM_NUMBER = 0;
    public static final int CAM_EXPOSURE = 5;
    public static final int IMG_WIDTH = 320;
    public static final int IMG_HEIGHT = 240;
    public static final int HOR_FOV_DEGREES = 60;
    public static final double HOR_DEGREES_PER_PIXEL = (double) HOR_FOV_DEGREES / IMG_WIDTH;
    public static final int IMG_HOR_MID = IMG_WIDTH / 2;
    public static final int DRIVER_STATION_FPS = 6;
    public static final int PROCESSING_FPS = 30; // DON'T CHANGE
    public static final int ANGLE_NOT_DETECTED = 600;
    public static final int ANGLE_RANGE = 5;
    public static final double HUB_TURN_SPEED = 0.3;

    // Colors are (B, G, R) (Don't ask me)
    public static final Scalar RED = new Scalar(0, 0, 255);
    public static final Scalar GREEN = new Scalar(0, 255, 0);
    public static final Scalar BLUE = new Scalar(255, 0, 0);
    public static final Scalar BLACK = new Scalar(0, 0, 0);

	public static final double CAMERA_OFFSET = 4;

    //Catapult Constants
    public static final double ACCEPTABLE_BAND_ERROR = 1500;

    // blaster PID constants
    public static final double BLASTER_KF = .054;
	public static final double BLASTER_KP = 1.1367;
	public static final double BLASTER_KI = 0;
    public static final double BLASTER_KD = 45.4667;

    private static final double KNOWN_LIDAR_DISTANCE_TO_TARGET = 120;
    private static final double KNOWN_OFFSET_ANGLE = Math.toRadians(4.45);
    public static final double SHOOTER_OFFSET_DISTANCE = Constants.KNOWN_LIDAR_DISTANCE_TO_TARGET*Math.tan(Constants.KNOWN_OFFSET_ANGLE);
	
    //gyro PID constants
	public static final double GYRO_kP = 0.0352; //0.1323; //0.1240
	public static final double GYRO_kI = 0;
	public static final double GYRO_kD = 0.01271; // 0.0479; //0.0572
	public static final double GYRO_TOLERANCE = 0.5; //degrees tolerance for measurement
	public static final double GYRO_RATE_TOLERANCE_DEG_PER_SEC = 10; // degrees per second
    public static final float GYRO_TARGET_ANGLE = 0;
	public static final boolean GYRO_REVERSED = false;
    
    //auton constants    
    public static final double ENCODER_TICKS_PER_INCH = 3500;

	public static final double AUTON_PUSH_ROBOT_DISTANCE = 4*ENCODER_TICKS_PER_INCH;
    public static final double AUTON_FORWARD_BALL_PICKUP_DISTANCE = 7500 + 644;
    public static final double AUTON_DRIVE_FORWARD_DISTANCE = 500;
    public static final double AUTON_DRIVE_FORWARD_SPEED = .5;
    //the following are approximate and in feet
	public static final double SNAG_N_YEET_DISTANCE_TO_TRENCH = 8.888889*12*ENCODER_TICKS_PER_INCH;
	public static final double SNAG_N_YEET_DISTANCE_ACROSS_FIELD = 17.77778*12*ENCODER_TICKS_PER_INCH;
    public static final double AUTON_PUSH_ROBOT_SPEED = .5;
    
    
	public static final double YEET3PUSHNOM3_DIAG_DISTANCE_TO_TRENCH = 60*ENCODER_TICKS_PER_INCH;
	public static final double YEET3PUSHNOM3_3_BALL_STRAIGHT_DISTANCE = 60*ENCODER_TICKS_PER_INCH;
	public static final double AUTON_2_BALL_STRAIGHT_DISTANCE = 124*ENCODER_TICKS_PER_INCH;
    public static final double AUTON_1_BALL_STRAIGHT_DISTANCE = 36*ENCODER_TICKS_PER_INCH;
    
    public static final double AUTON_45_TURN_CENTER_DISTANCE = 96*ENCODER_TICKS_PER_INCH;
    
    public static final double AUTON_2_BALL_RP_STRAIGHT_DISTANCE = 0;
    
    public static final double AUTON_DRIVE_TO_1ST_BALL = 32*ENCODER_TICKS_PER_INCH;
	public static final double AUTON_DRIVE_OFF_LINE_SPEED = .5;
    public static final double AUTON_DRIVE_OFF_LINE_DISTANCE = 12*ENCODER_TICKS_PER_INCH;
    
    // CALIBRATE THESE
	public static final double AUTON_TARGET_CENTER_LINE_CONSTANT_VELOCITY = 10343;
	public static final double TRENCH_SHOOTER_VELOCITY = 10343;
    public static final boolean BALL_COLOR_DETECTION = false;
}
