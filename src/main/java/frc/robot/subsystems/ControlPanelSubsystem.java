/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.ColorSensor;
import frc.robot.utils.Log;

import java.util.Arrays;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Color Sensor dependencies:
import edu.wpi.first.wpilibj.util.Color;

//FMS dependencies:
import edu.wpi.first.wpilibj.DriverStation;

//Motor dependencies:
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

public class ControlPanelSubsystem extends SubsystemBase {
  
  private Color lastColor;
  public BaseMotorController panelMotor;
  private boolean rotationsFinished;
  private boolean alignerFinished;

  private ColorSensor colorSensor;
  
  /**
   * Creates a new ControlPanel Subsystem.
   */
  public ControlPanelSubsystem(BaseMotorController panelMotor, ColorSensor colorSensor){
    super();

    this.colorSensor = colorSensor;
    lastColor = Constants.UNKNOWN_TARGET; //using Black as Unknown color
    rotationsFinished = false;
    alignerFinished = false;

    this.panelMotor = panelMotor; //new WPI_TalonSRX(Constants.PANEL_MOTOR_PORT);

    Log.info("Initiating Control Panel Subsystem");
  }

  //Gets the color that our color sensor currently detects
  public Color getColor() {

    final Color detectedColor = colorSensor.getColor();

    //  Run the color match algorithm on our detected color
    Color currentColor = colorSensor.matchClosestColor(detectedColor);

    if (!currentColor.equals(Constants.BLUE_TARGET) && !currentColor.equals(Constants.RED_TARGET) && !currentColor.equals(Constants.GREEN_TARGET) && !currentColor.equals(Constants.YELLOW_TARGET))
        currentColor = Constants.UNKNOWN_TARGET;    

    //If beyond 3 inches, detected color is inaccurate
    if (getProximity() < 100) {
        currentColor = Constants.UNKNOWN_TARGET;
    }

    //Account for inaccurate colors detected in transition between two colors
    if (!mapNextColor(lastColor).equals(currentColor) && !lastColor.equals(Constants.UNKNOWN_TARGET)) {
        //If the detected color does not appropriately match the predicted color to come after the last color,
        //And both the last color and direction clockwise/counter-clockwise are known,
        //Then stick to the value of the lastColor.
        currentColor = lastColor;
    }

    lastColor = currentColor;

    //Log.infoF("Current color: %s", currentColor.toString());
    return currentColor;
}

//Predicts the next color our sensor will detect given our current color and the direction we are spinning
public Color mapNextColor(Color color) {
    final Color currentColor = color;
    final Constants.DIRECTIONS currentDirection = Constants.SPIN_DIRECTION; // direction team has decided to spin in
    Color nextColor;

    final int index = Arrays.asList(Constants.COLOR_ORDER).indexOf(currentColor);
    int newIndex;

    if (currentDirection == Constants.DIRECTIONS.CLOCKWISE) {
        if (index + 1 > Constants.COLOR_ORDER.length - 1) {
            newIndex = 0;
        }
        else {
            newIndex = index + 1;
        }

        nextColor = Constants.COLOR_ORDER[newIndex];
    }
    else if (currentDirection == Constants.DIRECTIONS.COUNTERCLOCKWISE) {
        if (index - 1 < 0) {
            newIndex = Constants.COLOR_ORDER.length - 1;
        }
        else {
            newIndex = index - 1;
        }

        nextColor = Constants.COLOR_ORDER[newIndex];
    }
    else {
        nextColor = Constants.UNKNOWN_TARGET;
    }

    return nextColor;
}

//This method gets the proximity of the color sensor to the wheel
public double getProximity() {
    return colorSensor.getProximity();
}

//Starts spinning
public void startMotor(Modes mode) {

    //Log.infoF("Start Motor: Mode = %s, %% Output = %f", Modes.Rotations, mode == Modes.Rotations? Constants.PANEL_MOTOR_SPEED_ROTATION : Constants.PANEL_MOTOR_SPEED_ALIGNMENT);
    panelMotor.set(ControlMode.PercentOutput, mode == Modes.Rotations? Constants.PANEL_MOTOR_SPEED_ROTATION : Constants.PANEL_MOTOR_SPEED_ALIGNMENT);
}

//Stops spinning
public void stopMotor() {

    Log.info("Stopping Motor");
    panelMotor.set(ControlMode.PercentOutput, 0.0);
}


private Color rotationalStartingColor;
private int rotationalColorCount;
private boolean offStartingColor = false;

//For the first phase of the control panel: rotating the wheel 3-5 times
public void startRotations(Modes mode) {
    //We can use the color our sensor is detecting as opposed to the game's sensor, it will still work:
    rotationalStartingColor = getColor();
    rotationalColorCount = 0;
    offStartingColor = false;
    rotationsFinished = false;

    SmartDashboard.putBoolean("Start Rotations", true);
    //start rotating control wheel motor
    startMotor(mode);
}

//This method returns the number of rotations of the color wheel
public int getRotations() {
    return rotationalColorCount;
}

//This method monitors the rotations of the wheel and stops rotating once 3 rotations are complete
public void monitorRotations() {

    //Log.infoF("Rotational color count: %d", rotationalColorCount);
    SmartDashboard.putNumber("Rotational Color Count", rotationalColorCount);

    if (rotationalStartingColor.equals(getColor()) && offStartingColor) {
        rotationalColorCount++;
        offStartingColor = false;
    }
    if (!rotationalStartingColor.equals(getColor())) {
        offStartingColor = true;
    }

    if (getRotations() >= Constants.MIN_ROTATIONS) {
        //stop rotating
        rotationsFinished = true;
        stopMotor();

        Log.info("Rotations stopped");
        SmartDashboard.putBoolean("Start Rotations", false);

    }
}

public boolean isFinished(Modes event) {
    if (event == Modes.Rotations) {
        return rotationsFinished;
    }
    else if(event == Modes.Alignment) {
        return alignerFinished;
    }
    else {
        return false;
    }
}

//Gets color provided by FMS
public Color getFmsColor() {
    String gameData;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    Color retVal = Constants.UNKNOWN_TARGET;

    if (gameData.length() > 0) {
        switch (gameData.charAt(0)) {
        case 'B':
            retVal = Constants.BLUE_TARGET;
            break;
        case 'G':
            retVal = Constants.GREEN_TARGET;
            break;
        case 'R':
            retVal = Constants.RED_TARGET;
            break;
        case 'Y':
            retVal = Constants.YELLOW_TARGET;
            break;
        default:
            // This is corrupt data
            retVal = Constants.UNKNOWN_TARGET;
        }
    }

    //Log.infoF("FMS Color: %c", gameData.charAt(0));
    return retVal;
}


private Color targetColor;

/* For the second/final phase of control panel: aligning the game's sensor to a specific color on the panel */
public void startAligner(Modes mode) {
    targetColor = getFmsColor();
    alignerFinished = false;

    SmartDashboard.putString("FMS Color", targetColor.toString());
    SmartDashboard.putBoolean("Start Alignment", true);

    Log.info("Start aligning");

    //Start rotating
    startMotor(mode);
}

//Gets the distance, in color slices, between our sensor and the game sensor
public int getDifferential() {
    return Constants.DIFFERENTIAL;
}

//Gets the color that is two (or whatever the differential is) color slices away from our sensor's position
public Color colorCorrector(Color currentColor) {
    if (getDifferential() == 2) {
        return mapNextColor(mapNextColor(currentColor));
    }
    else if (getDifferential() == 3) {
        return mapNextColor(mapNextColor(mapNextColor(currentColor)));
    }
    else {
        return mapNextColor(currentColor);
    }
}

//Stops rotating once the correct color is reached
public void monitorAligner() {
    boolean isTarget = false;
    
    if (targetColor.equals(colorCorrector(getColor()))) {
        isTarget = true;
    }
    else {
        isTarget = false;
    }

    if (isTarget) {
        stopMotor();
        alignerFinished = true;

        Log.info("Alignment completed");
        SmartDashboard.putBoolean("Start Alignment", false);

    }
}

//This method puts stuff on the dashboard (only use if necessary) for first phase
public void putDashRotations() {

    SmartDashboard.putString("Detected Color", getColor().toString());
    SmartDashboard.putString("Predicted Next Color", mapNextColor(getColor()).toString());
    SmartDashboard.putNumber("Rotations", getRotations());

}

//This method puts stuff on the dashboard (only use if necessary) for second phase
public void putDashAligner() {

    SmartDashboard.putString("Detected Color", getColor().toString());
    SmartDashboard.putString("Predicted Gamesensor Color", colorCorrector(getColor()).toString());
    SmartDashboard.putString("Target Gamesensor Color", targetColor.toString());

}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static enum Modes{
    Rotations,
    Alignment
  }
}
