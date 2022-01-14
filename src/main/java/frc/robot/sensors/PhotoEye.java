/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Add your docs here.
 */
public class PhotoEye extends BaseSensor {
    private DigitalInput photoEye;
    public PhotoEye (int port){
        this.photoEye = new DigitalInput(port);
    }
    public boolean isDetecting(){
        return photoEye.get(); 
    }

    public DigitalInput getDigitalInput()
    {
        return photoEye;
    }

    // public static void main(String[] args) {
    //     PhotoEye pe1 = new PhotoEye(6);
    //     PhotoEye pe2 = new PhotoEye(7);

    //     while (true){
    //         System.out.println("PhotoEye1 Detecting: " + pe1.isDetecting());
    //         System.out.println("PhotoEye2 Detecting: " + pe2.isDetecting());
    //     }
    // }
}