
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.PhotoEye;
import frc.robot.utils.Log;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class HangSubsystem extends SubsystemBase {

    private boolean hangIsOut = false;
    private String direction = "stop";
    private DoubleSolenoid hangClamp;
    private DoubleSolenoid hangAngle;
    public BaseMotorController hangMotor;
    public BaseMotorController winchMotor1;
    public BaseMotorController winchMotor2;
    private DigitalInput leftRungSwitch;
    private DigitalInput rightRungSwitch;
    private DigitalInput rightClampedSwitch;
    private DigitalInput leftClampedSwitch;
    private PhotoEye rungArmAdvancedSwitch;
    private PhotoEye rungArmRetractedSwitch;
    private PhotoEye rungArmMidSwitch;
    private Boolean suppressClamp;
    private double motorSpeed = 0;

    public HangSubsystem(BaseMotorController hangMotor, BaseMotorController winchMotor1, BaseMotorController winchMotor2, 
        DigitalInput leftRungSwitch, DigitalInput rightRungSwitch, PhotoEye rungArmRetractedSwitch, PhotoEye rungArmMidSwitch, 
        PhotoEye rungArmAdvancedSwitch, DoubleSolenoid hangAngle, DoubleSolenoid hangClamp, DigitalInput rightClampedSwitch, DigitalInput leftClampedSwitch) {

        super();

        this.hangMotor = hangMotor; 
        this.winchMotor1 = winchMotor1;
        this.winchMotor2 = winchMotor2;

        this.hangAngle = hangAngle;
        this.hangClamp = hangClamp;

        this.leftRungSwitch = leftRungSwitch;
        this.rightRungSwitch = rightRungSwitch;
        this.rungArmAdvancedSwitch = rungArmAdvancedSwitch;
        this.rungArmRetractedSwitch = rungArmRetractedSwitch;
        this.rungArmMidSwitch = rungArmMidSwitch;
        this.leftClampedSwitch = leftClampedSwitch;
        this.rightClampedSwitch = rightClampedSwitch;

        this.hangClamp = hangClamp;

        initialize();

        Log.info("Initializing Hang Subsystem");
    }

    private void initialize () {
        winchMotor1.setNeutralMode(NeutralMode.Brake);
        winchMotor2.setNeutralMode(NeutralMode.Brake);
        suppressClamp = false;
        motorSpeed = 0;
        hangIsOut = false;
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

    public void setSuppressClamp (Boolean state) {
        suppressClamp = state;
    }

    public void winchDown() { //Goes Up
        Log.infoF("Winch going up - Speed: %f", Constants.WINCH_SPEED);
        if (!rungArmAdvancedSwitch.isDetecting()) {
            direction = "up";
            motorSpeed = Constants.WINCH_SPEED;
            // winchMotor2.set(ControlMode.PercentOutput, Constants.WINCH_SPEED);
            // winchMotor1.set(ControlMode.PercentOutput, Constants.WINCH_SPEED);
        }
        else
            motorSpeed = 0;
    }

    public void winchUp() { //Pulls Down
        Log.infoF("Winch going down - Speed: %f", Constants.WINCH_SPEED);
        if (!rungArmRetractedSwitch.isDetecting()) {
            direction = "down";
            motorSpeed = -(Constants.WINCH_SPEED);
            // winchMotor2.set(ControlMode.PercentOutput, -(Constants.WINCH_SPEED));
            // winchMotor1.set(ControlMode.PercentOutput, -(Constants.WINCH_SPEED));
        }
        else
            motorSpeed = 0;
    }    

    public void winchOff() {
        Log.info("Stopping Winch ...");
        direction = "stop";
        motorSpeed = 0;
        // winchMotor2.set(ControlMode.PercentOutput, 0);
        // winchMotor1.set(ControlMode.PercentOutput, 0);
    }

    public void grabRung(){
        Log.info("Rung Grabbed");
        hangClamp.set(Value.kReverse);
    }

    public void ungrabRung(){
        Log.info("Ungrab Rung");
        hangClamp.set(Value.kForward);
    }

    public void hangOut(){
        Log.info("Hang out initiated");
        hangAngle.set(Value.kForward);
        hangIsOut = true;
    }

    public void hangIn(){
        Log.info("Hang In initiated");
        hangAngle.set(Value.kReverse);
        hangIsOut = false;
    }

    public boolean isHangOut(){
        return hangIsOut;
    }

    @Override
    public void periodic() {

        SmartDashboard.putBoolean("Retracted Switch", rungArmRetractedSwitch.isDetecting());
        SmartDashboard.putBoolean("Mid Switch", rungArmMidSwitch.isDetecting());
        SmartDashboard.putBoolean("Adv Switch", rungArmAdvancedSwitch.isDetecting());
        
        if(direction == "down" && rungArmRetractedSwitch.isDetecting() == true){
            Log.info("Going down and arm retracted. Stopping Winch Motors");
            System.out.println("Going down and arm retracted. Stopping Winch Motors");
            winchOff();
        }
        if(direction == "up" && hangIsOut == false && rungArmMidSwitch.isDetecting() == true){
            Log.info("Going up, hang In and arm at mid point. Stopping Winch Motors");
            System.out.println("Going up, hang In and arm at mid point. Stopping Winch Motors");
            // System.out.println("Run Arm Mid Switch" + rungArmMidSwitch.isDetecting());
            winchOff();
        }
        if(direction == "up" && rungArmAdvancedSwitch.isDetecting() == true){
            Log.info("Going up, hang not out and arm at advanced point. Stopping Winch Motors");
            System.out.println("Going up, hang not out and arm at advanced point. Stopping Winch Motors");
            winchOff();
        }

        //clamp if the rung is in place on both sides 
        Boolean clampedDown = (rightRungSwitch.get() && leftRungSwitch.get());
        if (leftClampedSwitch.get() && rightClampedSwitch.get() && !clampedDown) {
            grabRung();
            clampedDown = true;
            // System.out.println("Clamping");
        }

        winchMotor2.set(ControlMode.PercentOutput, motorSpeed);
        winchMotor1.set(ControlMode.PercentOutput, motorSpeed);

        // System.out.println("Left Rung Clamped: " + leftRungSwitch.get() + " | Right Rung Clamped: " + rightRungSwitch.get() + " | Clamped: " + clampedDown + " | Direction: " + direction);
        //System.out.println("Retracted: " + rungArmRetractedSwitch.isDetecting() + " | Mid: " + rungArmMidSwitch.isDetecting() + " | Extended: " + rungArmAdvancedSwitch.isDetecting() + " | Clamped: " + clampedDown);
    }
}
