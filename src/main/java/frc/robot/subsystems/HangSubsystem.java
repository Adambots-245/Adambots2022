
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Log;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class HangSubsystem extends SubsystemBase {

    // TODO Add hangmotor Constant
    public WPI_VictorSPX hangMotor;
    public WPI_VictorSPX winchMotor1;
    public WPI_VictorSPX winchMotor2;
    private DigitalInput limitSwitch1;
    private DigitalInput limitSwitch2;

    public HangSubsystem(WPI_VictorSPX hangMotor, WPI_VictorSPX winchMotor1, WPI_VictorSPX winchMotor2, DigitalInput limitSwitch1, DigitalInput limitSwitch2) {
        super();

        this.hangMotor = hangMotor; // new WPI_VictorSPX(Constants.CLIMBING_RAISE_ELEVATOR_MOTOR_PORT);
        this.winchMotor1 = winchMotor1; //  new WPI_VictorSPX(Constants.CLIMBING_1_MOTOR_PORT);
        this.winchMotor2 = winchMotor2; // new WPI_VictorSPX(Constants.CLIMBING_2_MOTOR_PORT);

        this.winchMotor2.setInverted(true);
        this.limitSwitch1 = limitSwitch1; // new DigitalInput(Constants.ELEVATOR_LIMIT_SWITCH_1_PORT);
        this.limitSwitch2 = limitSwitch2; // new DigitalInput(Constants.ELEVATOR_LIMIT_SWITCH_2_PORT);

        Log.info("Initializing Hang Subsystem");
    }

    public void climb(double speed, boolean overrideFlag) {
        if (!limitSwitch1.get() && !limitSwitch2.get() && speed >= 0 && !overrideFlag) {
          
            Log.info("Stopping climb. Either limit switch hit or speed set to 0");
            hangMotor.set(ControlMode.PercentOutput, Constants.STOP_MOTOR_SPEED);
        } else {

            //Log.infoF("Starting climb - Speed: %f", speed);
            hangMotor.set(ControlMode.PercentOutput, speed);
        }

        // System.out.println(overrideButton.get());
    }

    public void winchDown() {

        //Log.infoF("Winch down - Speed: %f", Constants.WINCH_SPEED);
        winchMotor2.set(ControlMode.PercentOutput, Constants.WINCH_SPEED);
        winchMotor1.set(ControlMode.PercentOutput, Constants.WINCH_SPEED);
    }

    public void winchOff() {

        Log.info("Stopping Winch");
        winchMotor2.set(ControlMode.PercentOutput, 0);
        winchMotor1.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {

    }
}
