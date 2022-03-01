
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
<<<<<<< HEAD
    public boolean hangIsOut = false;
    private boolean goingDown = false;
    private DoubleSolenoid hangClamp;
    private DoubleSolenoid hangAngle;
    public WPI_VictorSPX hangMotor;
    public BaseMotorController winchMotor1;
    public BaseMotorController winchMotor2;
    public WPI_VictorSPX angleClimbMotor;
    public WPI_VictorSPX HangAngleMotor1;
    public WPI_VictorSPX HangAngleMotor2;
    //private DigitalInput limitSwitch1;
    //private DigitalInput limitSwitch2;
    private DigitalInput leftRungSwitch;
    private DigitalInput rightRungSwitch;
    private DigitalInput rungArmAdvancedSwitch;
    private DigitalInput rungArmRetractedSwitch;
    private DigitalInput rungArmMidSwitch;

    public HangSubsystem(WPI_VictorSPX hangMotor, BaseMotorController winchMotor1, BaseMotorController winchMotor2, WPI_VictorSPX hangAngleMotor1, WPI_VictorSPX hangAngleMotor2, DigitalInput leftRungSwitch, DigitalInput rightRungSwitch, DigitalInput rungArmRetractedSwitch, DigitalInput rungArmMidSwitch, DigitalInput rungArmAdvancedSwitch) {
=======
    private WPI_VictorSPX hangMotor;
    private WPI_VictorSPX winchMotor1;
    private WPI_VictorSPX winchMotor2;
    private DigitalInput limitSwitch1;
    private DigitalInput limitSwitch2;
    private DoubleSolenoid rungClamp;

    public HangSubsystem(WPI_VictorSPX hangMotor, WPI_VictorSPX winchMotor1, WPI_VictorSPX winchMotor2, DigitalInput limitSwitch1, DigitalInput limitSwitch2, DoubleSolenoid rungClamp) {
>>>>>>> 42f9b19b705b02308017362c205273bbeac86c01
        super();

        this.hangMotor = hangMotor; // new WPI_VictorSPX(Constants.CLIMBING_RAISE_ELEVATOR_MOTOR_PORT);
        this.winchMotor1 = winchMotor1; //  new WPI_VictorSPX(Constants.CLIMBING_1_MOTOR_PORT);
        this.winchMotor2 = winchMotor2; // new WPI_VictorSPX(Constants.CLIMBING_2_MOTOR_PORT);
<<<<<<< HEAD
        //this.angleClimbMotor = angleClimbMotor;

        this.winchMotor2.setInverted(true);
        //this.limitSwitch1 = limitSwitch1; // new DigitalInput(Constants.ELEVATOR_LIMIT_SWITCH_1_PORT);
        //this.limitSwitch2 = limitSwitch2; // new DigitalInput(Constants.ELEVATOR_LIMIT_SWITCH_2_PORT);
=======
        this.winchMotor2.setInverted(true);

        this.limitSwitch1 = limitSwitch1; // new DigitalInput(Constants.ELEVATOR_LIMIT_SWITCH_1_PORT);
        this.limitSwitch2 = limitSwitch2; // new DigitalInput(Constants.ELEVATOR_LIMIT_SWITCH_2_PORT);
>>>>>>> 42f9b19b705b02308017362c205273bbeac86c01

        this.rungClamp = rungClamp;

        Log.info("Initializing Hang Subsystem");
    }

    // public void climb(double speed, boolean overrideFlag) {
    //     if (!limitSwitch1.get() && !limitSwitch2.get() && speed >= 0 && !overrideFlag) {
          
<<<<<<< HEAD
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
=======
            Log.info("Stopping climb. Either limit switch hit or speed set to 0");
            hangMotor.set(ControlMode.PercentOutput, Constants.STOP_MOTOR_SPEED);
        } else {
            hangMotor.set(ControlMode.PercentOutput, speed);
        }
    }

    public void winchDown() {
>>>>>>> 42f9b19b705b02308017362c205273bbeac86c01
        winchMotor2.set(ControlMode.PercentOutput, Constants.WINCH_SPEED);
            winchMotor1.set(ControlMode.PercentOutput, Constants.WINCH_SPEED);
        
    }

    public void winchUp() {
<<<<<<< HEAD
        goingDown = false;
                winchMotor2.set(ControlMode.PercentOutput, -(Constants.WINCH_SPEED));
                winchMotor1.set(ControlMode.PercentOutput, -(Constants.WINCH_SPEED));
        }    

    public void winchOff() {
=======
        winchMotor2.set(ControlMode.PercentOutput, -Constants.WINCH_SPEED);
        winchMotor1.set(ControlMode.PercentOutput, -Constants.WINCH_SPEED);
    }
>>>>>>> 42f9b19b705b02308017362c205273bbeac86c01

    public void winchOff() {
        winchMotor2.set(ControlMode.PercentOutput, 0);
        winchMotor1.set(ControlMode.PercentOutput, 0);
    }

    public void ChangeHangAngle(DoubleSupplier speedInput) {
        HangAngleMotor1.set(ControlMode.PercentOutput, speedInput.getAsDouble());
        HangAngleMotor2.set(ControlMode.PercentOutput, speedInput.getAsDouble());
    }

    public void GrabRung(){
        if(leftRungSwitch.get() && rightRungSwitch.get()){
            hangClamp.set(Value.kReverse);
            System.out.println("Grabbed");
        }
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
