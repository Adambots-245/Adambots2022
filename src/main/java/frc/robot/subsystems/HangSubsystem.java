
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.PhotoEye;
import frc.robot.utils.Log;

public class HangSubsystem extends SubsystemBase {

    private boolean hangIsOut = false;
    private DoubleSolenoid hangClamp;
    private DoubleSolenoid hangAngle;
    private BaseMotorController winchMotor1;
    private BaseMotorController winchMotor2;
    private DigitalInput leftRungSwitch;
    private DigitalInput rightRungSwitch;
    private DigitalInput rightClampedSwitch;
    private DigitalInput leftClampedSwitch;
    private PhotoEye rungArmAdvancedSwitch;
    private PhotoEye rungArmRetractedSwitch;
    private PhotoEye rungArmMidSwitch;
    private double winchSpeed = 0;

    public HangSubsystem(BaseMotorController winchMotor1, BaseMotorController winchMotor2, DigitalInput leftRungSwitch, 
        DigitalInput rightRungSwitch, PhotoEye rungArmRetractedSwitch, PhotoEye rungArmMidSwitch, PhotoEye rungArmAdvancedSwitch, 
        DoubleSolenoid hangAngle, DoubleSolenoid hangClamp, DigitalInput rightClampedSwitch, DigitalInput leftClampedSwitch) {

        super();

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
        winchMotor1.setInverted(true); //Invert both to make up positive and logic eaier to understand
        winchMotor2.setInverted(true);
        winchSpeed = 0;
        hangIsOut = false;
    }

    public void winchDown() { //Goes Up
        Log.infoF("Winch going up - Speed: %f", Constants.WINCH_SPEED);
        if (!rungArmAdvancedSwitch.isDetecting()) {
            winchSpeed = Constants.WINCH_SPEED;
        }
        else
            winchSpeed = 0;
    }

    public void winchUp() { //Pulls Down
        Log.infoF("Winch going down - Speed: %f", Constants.WINCH_SPEED);
        if (!rungArmRetractedSwitch.isDetecting()) {
            winchSpeed = -Constants.WINCH_SPEED;
        }
        else
            winchSpeed = 0;
    }    

    public void winchOff() {
        Log.info("Stopping Winch ...");
        winchSpeed = 0;
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
        
        if(winchSpeed < 0 && rungArmRetractedSwitch.isDetecting() == true){
            Log.info("Going down and arm retracted. Stopping Winch Motors");
            winchOff();
        }
        if(winchSpeed > 0 && hangIsOut == false && rungArmMidSwitch.isDetecting() == true){
            Log.info("Going up, hang In and arm at mid point. Stopping Winch Motors");
            winchOff();
        }
        if(winchSpeed > 0 && rungArmAdvancedSwitch.isDetecting() == true){
            Log.info("Going up, arm at advanced point. Stopping Winch Motors");
            winchOff();
        }

        //clamp if the rung is in place on both sides 
        Boolean clampedDown = (rightRungSwitch.get() || leftRungSwitch.get());
        if (leftClampedSwitch.get() && rightClampedSwitch.get() && !clampedDown) {
            grabRung();
            Log.info("Clamping");
        }

        winchMotor2.set(ControlMode.PercentOutput, winchSpeed);
        winchMotor1.set(ControlMode.PercentOutput, winchSpeed);

        // System.out.println("Left Rung Clamped: " + leftClampedSwitch.get() + " | Right Rung Clamped: " + rightClampedSwitch.get() + " | Clamped: " + clampedDown);
        //System.out.println("Retracted: " + rungArmRetractedSwitch.isDetecting() + " | Mid: " + rungArmMidSwitch.isDetecting() + " | Extended: " + rungArmAdvancedSwitch.isDetecting() + " | Clamped: " + clampedDown);
    }
}