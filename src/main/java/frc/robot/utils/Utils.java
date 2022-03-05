// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class Utils {

    public enum BallPosition{
        ONE,
        TWO,
        THREE,
        FOUR
    }

    public static int firstDistance(BallPosition ballPosition){
        int distance = 0;

        switch(DriverStation.getLocation()) {
            case 1:
                switch (ballPosition){
                    case ONE:
                        distance = 42;
                        break;
                     case TWO:
                         distance = 104;
                         break;
                    case THREE:
                        distance = 156;
                        break;
                    default:
                        break;
                }
                break;
            case 2:
            switch (ballPosition){
                case ONE:
                    distance = 30;
                    break;
            }
                break;
            case 3:
                switch (ballPosition){
                    case ONE:
                        distance = 42;
                        break;
                    default:
                        break;
                }
                break;
            default:
                System.out.println("couldn't find team");
                break;    
        }
        return distance;
    }
}
