
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Log;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class HangSubsystem extends SubsystemBase {

    private boolean hangIsOut = false;
    private boolean goingDown = false;
    private DoubleSolenoid hangClamp;
    private DoubleSolenoid hangAngle;
    public BaseMotorController hangMotor;
    public BaseMotorController winchMotor1;
    public BaseMotorController winchMotor2;
    private DigitalInput leftRungSwitch;
    private DigitalInput rightRungSwitch;
    private DigitalInput rungArmAdvancedSwitch;
    private DigitalInput rungArmRetractedSwitch;
    private DigitalInput rungArmMidSwitch;

    public HangSubsystem(BaseMotorController hangMotor, BaseMotorController winchMotor1, BaseMotorController winchMotor2, DigitalInput leftRungSwitch, DigitalInput rightRungSwitch, DigitalInput rungArmRetractedSwitch, DigitalInput rungArmMidSwitch, DigitalInput rungArmAdvancedSwitch, DoubleSolenoid hangAngle, DoubleSolenoid hangClamp) {

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

        //this.winchMotor2.setInverted(true);

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
        Log.infoF("Winch going down - Speed: %f", Constants.WINCH_SPEED);
        goingDown = true;
        winchMotor2.set(ControlMode.PercentOutput, Constants.WINCH_SPEED);
        winchMotor1.set(ControlMode.PercentOutput, Constants.WINCH_SPEED);
    }

    public void winchUp() {
        Log.infoF("Winch going up - Speed: %f", Constants.WINCH_SPEED);

        goingDown = false;
        winchMotor2.set(ControlMode.PercentOutput, -(Constants.WINCH_SPEED));
        winchMotor1.set(ControlMode.PercentOutput, -(Constants.WINCH_SPEED));
        }    


    public void winchOff() {
        Log.info("Stopping Winch ...");

        winchMotor2.set(ControlMode.PercentOutput, 0);
        winchMotor1.set(ControlMode.PercentOutput, 0);
    }

    public void grabRung(){
        // if(leftRungSwitch.get() && rightRungSwitch.get()){

            Log.info("Rung Grabbed");
            hangClamp.set(Value.kReverse);
            // System.out.println("Grabbed");
        // }
    }

    public void ungrabRung(){

        Log.info("Ungrab Rung");
        hangClamp.set(Value.kForward);
        // System.out.println("Ungrabbed");
    }

    public void hangOut(){

        Log.info("Hang out initiated");

        hangAngle.set(Value.kForward);
        // System.out.println("Hang Out");
    }

    public void hangIn(){
        Log.infoF("Rung Arm Retracted Swith Status: %b", rungArmRetractedSwitch.get());
        if(rungArmRetractedSwitch.get() == true){

            Log.info("Hang In initiated");
            hangAngle.set(Value.kReverse);
            // System.out.println("Hang In");
        }
    }

    public boolean isHangOut(){
        return hangIsOut;
    }

    public void setHangOut(boolean value){
        this.hangIsOut = value;
    }

    @Override
    public void periodic() {
        if(goingDown == true && rungArmRetractedSwitch.get() == true){
            Log.info("Going down and run arm retracted. Stopping Winch Motors");
          
            winchMotor2.set(ControlMode.PercentOutput, 0);
            winchMotor1.set(ControlMode.PercentOutput, 0);
        }
        if(goingDown == false && hangIsOut == false && rungArmMidSwitch.get() == true){
            Log.info("Not Going down, no hang out and arm at mid point. Stopping Winch Motors");

            winchMotor2.set(ControlMode.PercentOutput, 0);
            winchMotor1.set(ControlMode.PercentOutput, 0);
        }
        if(goingDown == false && hangIsOut == true && rungArmAdvancedSwitch.get() == true){
            Log.info("Not Going down, hang out true and arm at advanced point. Stopping Winch Motors");

            winchMotor2.set(ControlMode.PercentOutput, 0);
            winchMotor1.set(ControlMode.PercentOutput, 0);
        }
    }
}
