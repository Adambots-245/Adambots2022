
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Log;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


import com.ctre.phoenix.motorcontrol.ControlMode;

public class HangSubsystem extends SubsystemBase {

    // TODO Add hangmotor Constant
    public boolean hangIsOut = false;
    private boolean goingDown = false;
    private DoubleSolenoid hangClamp;
    private DoubleSolenoid hangAngle;
    public WPI_VictorSPX hangMotor;
    public BaseMotorController winchMotor1;
    public BaseMotorController winchMotor2;
    // public WPI_VictorSPX angleClimbMotor;
    //private DigitalInput limitSwitch1;
    //private DigitalInput limitSwitch2;
    private DigitalInput leftRungSwitch;
    private DigitalInput rightRungSwitch;
    private DigitalInput rungArmAdvancedSwitch;
    private DigitalInput rungArmRetractedSwitch;
    private DigitalInput rungArmMidSwitch;

    public HangSubsystem(WPI_VictorSPX hangMotor, BaseMotorController winchMotor1, BaseMotorController winchMotor2, DigitalInput leftRungSwitch, DigitalInput rightRungSwitch, DigitalInput rungArmRetractedSwitch, DigitalInput rungArmMidSwitch, DigitalInput rungArmAdvancedSwitch, DoubleSolenoid hangAngle, DoubleSolenoid hangClamp) {

        super();

        this.hangMotor = hangMotor; // new WPI_VictorSPX(Constants.CLIMBING_RAISE_ELEVATOR_MOTOR_PORT);
        this.winchMotor1 = winchMotor1; //  new WPI_VictorSPX(Constants.CLIMBING_1_MOTOR_PORT);
        this.winchMotor2 = winchMotor2; // new WPI_VictorSPX(Constants.CLIMBING_2_MOTOR_PORT);

        this.hangAngle = hangAngle;
        this.hangClamp = hangClamp;
        //this.angleClimbMotor = angleClimbMotor;

        this.leftRungSwitch = leftRungSwitch;
        this.rightRungSwitch = rightRungSwitch;
        this.rungArmAdvancedSwitch = rungArmAdvancedSwitch;
        this.rungArmRetractedSwitch = rungArmRetractedSwitch;
        this.rungArmMidSwitch = rungArmMidSwitch;

        //this.winchMotor2.setInverted(true);
        //this.limitSwitch1 = limitSwitch1; // new DigitalInput(Constants.ELEVATOR_LIMIT_SWITCH_1_PORT);
        //this.limitSwitch2 = limitSwitch2; // new DigitalInput(Constants.ELEVATOR_LIMIT_SWITCH_2_PORT);

        this.hangClamp = hangClamp;

        Log.info("Initializing Hang Subsystem");
    }

    // public void climb(double speed, boolean overrideFlag) {
    //     if (!limitSwitch1.get() && !limitSwitch2.get() && speed >= 0 && !overrideFlag) {
          
    //         Log.info("Stopping climb. Either limit switch hit or speed set to 0");
    //         hangMotor.set(ControlMode.PercentOutput, Constants.STOP_MOTOR_SPEED);
    //     } else {

    //         //Log.infoF("Starting climb - Speed: %f", speed);
    //         hangMotor.set(ControlMode.PercentOutput, speed);
    //     }

    //     // System.out.println(overrideButton.get());
    // }

    //public void changeClimbAngle(double angle, double speed){
    //    angleClimbMotor
    //}

    public void winchDown() {
        goingDown = true;
        winchMotor2.set(ControlMode.PercentOutput, Constants.WINCH_SPEED);
        winchMotor1.set(ControlMode.PercentOutput, Constants.WINCH_SPEED);
    }

    public void winchUp() {
        goingDown = false;
                winchMotor2.set(ControlMode.PercentOutput, -(Constants.WINCH_SPEED));
                winchMotor1.set(ControlMode.PercentOutput, -(Constants.WINCH_SPEED));
        }    


    public void winchOff() {
        winchMotor2.set(ControlMode.PercentOutput, 0);
        winchMotor1.set(ControlMode.PercentOutput, 0);
    }

    // public void ChangeHangAngle(DoubleSupplier speedInput) {
    //     HangAngleMotor1.set(ControlMode.PercentOutput, speedInput.getAsDouble());
    //     HangAngleMotor2.set(ControlMode.PercentOutput, speedInput.getAsDouble());
    // }

    public void GrabRung(){
        // if(leftRungSwitch.get() && rightRungSwitch.get()){
            hangClamp.set(Value.kReverse);
            System.out.println("Grabbed");
        // }
    }

    public void UngrabRung(){
        hangClamp.set(Value.kForward);
        System.out.println("Ungrabbed");
    }

    public void HangOut(){
        hangAngle.set(Value.kForward);
        System.out.println("Hang Out");
    }

    public void HangIn(){
        if(rungArmRetractedSwitch.get() == true){
            hangAngle.set(Value.kReverse);
            System.out.println("Hang In");
        }
    }

    @Override
    public void periodic() {
        if(goingDown == true && rungArmRetractedSwitch.get() == true){
            winchMotor2.set(ControlMode.PercentOutput, 0);
            winchMotor1.set(ControlMode.PercentOutput, 0);
        }
        if(goingDown == false && hangIsOut == false && rungArmMidSwitch.get() == true){
            winchMotor2.set(ControlMode.PercentOutput, 0);
            winchMotor1.set(ControlMode.PercentOutput, 0);
        }
        if(goingDown == false && hangIsOut == true && rungArmAdvancedSwitch.get() == true){
            winchMotor2.set(ControlMode.PercentOutput, 0);
            winchMotor1.set(ControlMode.PercentOutput, 0);
        }
    }
}
