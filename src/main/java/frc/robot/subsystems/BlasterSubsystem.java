package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Log;

public class BlasterSubsystem extends SubsystemBase {
    /**
     * Creates a new Intake.
     */

    private BaseMotorController blasterMotor; // WPI_TalonFX
    private Solenoid backboard;

    public BlasterSubsystem(BaseMotorController blasterMotor, Solenoid backboard) {
        super();

        this.blasterMotor = blasterMotor; // new WPI_TalonFX(Constants.BLASTER_MOTOR_PORT);
        this.backboard = backboard; // new Solenoid(Constants.RAISE_BLASTER_HOOD_SOL_PORT);

        if (blasterMotor != null) {
            Log.info("Blaster Subsystem initialized.");
            //Log.infoF("Blaster kF=[%f], kP=[%f], kI=[%f], kD=[%f]", Constants.BLASTER_KF, Constants.BLASTER_KP, Constants.BLASTER_KI, Constants.BLASTER_KD);
            
            blasterMotor.config_kF(0, Constants.BLASTER_KF);
            blasterMotor.config_kP(0, Constants.BLASTER_KP);
            blasterMotor.config_kI(0, Constants.BLASTER_KI);
            blasterMotor.config_kD(0, Constants.BLASTER_KD);
        }
    }

    // sets blaster wheel speed as a percentage output value (-1 to 1, 0 to stop)
    public void output(double speed) {

        //Log.infoF("Output: %% Output = %f", speed);
        blasterMotor.set(ControlMode.PercentOutput, speed);
    }

    // sets blaster wheel speed as a closed loop velocity value = position change /
    // 100ms
    public void setVelocity(double speed) {

        //Log.infoF("Set Velocity: Speed = %f", speed);

        blasterMotor.set(ControlMode.Velocity, speed);
    }

    // get current velocity in raw sensor units per 100ms.
    public int getVelocity() {

        // SmartDashboard.putNumber("Current", blasterMotor.getSupplyCurrent());
        return blasterMotor.getSelectedSensorVelocity();
    }

    /*
     * if backboard solenoid is true, the hood is raised and the ball will be able
     * to travel far. if backboard solenoid is false, the hood is lowered and the
     * ball has a more vertical trajectory (near shooting).
     */
    public boolean getBackboardPosition() {
        return backboard.get();
    }

    /*
     * if backboard solenoid is true, the hood is raised and the ball will be able
     * to travel far. if backboard solenoid is false, the hood is lowered and the
     * ball has a more vertical trajectory (near shooting).
     */
    public void setBackboard(boolean isBackboardFarPosition) {

        //Log.infoF("Set Backboard: isBackboardFarPosition = %b", isBackboardFarPosition);

        backboard.set(isBackboardFarPosition);
    }

    public void periodic() {
        // This method will be called once per scheduler run

        // NOT USED

    }

}