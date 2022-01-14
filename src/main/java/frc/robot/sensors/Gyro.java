/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;

/**
 * Class that represents a Gyroscope sensor
 */
public class Gyro extends BaseSensor implements edu.wpi.first.wpilibj.interfaces.Gyro {

    private static Gyro _instance = null;
    private static AHRS _navx = null;

    private static class InstanceHolder {
        public static final Gyro instance = new Gyro();
    }

    private Gyro(){
        try {
            if (_navx == null){
                _navx = new AHRS(); // although this brings in depency, using setDevice this can be overwritten before calling getInstance
            }

            _navx.enableBoardlevelYawReset(true);
            _navx.reset();
        } catch (Exception e) {
            System.out.println("NavX Initialization Failed");
        }
    }

    public static void setDevice(AHRS navx){
        
        if (_navx == null)
            _navx = navx; // It should only be set once.
    }

    public static Gyro getInstance(){
        // if (_instance == null){
        //     _instance = new Gyro();
        // }
        _instance = InstanceHolder.instance;
        return _instance;
    }

    public void reset(){
        _navx.reset();
        // _navx.enableBoardlevelYawReset(true);
    }

    public void lowLevelReset(){
        _navx.enableBoardlevelYawReset(true);
        _navx.reset();
    }

    /**
     * Use only in Robot Init
     */
    public void calibrationCheck() {

        boolean isCalibrating = _navx.isCalibrating();
        
        if (isCalibrating) {
            System.out.println("In Calibration Check - waiting 2 seconds to complete Gyro Calibration");
            Timer.delay(2); // wait 2 seconds to let it complete calibration
        }
    }

    public float getRoll() {
        return _navx.getRoll();
    }

    public float getPitch() {
        return _navx.getPitch();
    }

    /**
     * Returns current Yaw value between a range of -180 to 180 degrees. 
     * Note that this Yaw value can accumulate errors over a period of time.
     * @return
     */
    public float getYaw() {
        return _navx.getYaw();
    }

    @Override
    public void close() throws Exception {
        _navx.close();

    }

    /**
     * Returns accumulated Yaw angles and can be > than 360 degrees. This is in contrast to getYaw that returns -180 to 180 degree values.
     */
    @Override
    public double getAngle() {
        return _navx.getAngle();
    }

    @Override
    public double getRate() {
        return _navx.getRate();
    }

    @Override
    public void calibrate() {
        // do nothing - NavX automatically calibrates at startup
        
    }
}
