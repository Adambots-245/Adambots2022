/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Gamepad;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * All Game Controller Button Mappings
 */
public class Buttons {

    // initialize controllers
    public static final XboxController primaryJoystick = new XboxController(GamepadConstants.PRIMARY_DRIVER);
    public static final XboxController secondaryJoystick = new XboxController(GamepadConstants.SECONDARY_DRIVER);

    //primary buttons
    public static final JoystickButton primaryBackButton = new JoystickButton(primaryJoystick, GamepadConstants.BUTTON_BACK);
    public static final JoystickButton primaryStartButton = new JoystickButton(primaryJoystick, GamepadConstants.BUTTON_START);
    public static final JoystickButton primaryXButton = new JoystickButton(primaryJoystick, GamepadConstants.BUTTON_X);
    public static final JoystickButton primaryYButton = new JoystickButton(primaryJoystick, GamepadConstants.BUTTON_Y);
    public static final JoystickButton primaryBButton = new JoystickButton(primaryJoystick, GamepadConstants.BUTTON_B);
    public static final JoystickButton primaryAButton = new JoystickButton(primaryJoystick, GamepadConstants.BUTTON_A);
    public static final JoystickButton primaryLB = new JoystickButton(primaryJoystick, GamepadConstants.BUTTON_LB);
    public static final JoystickButton primaryRB = new JoystickButton(primaryJoystick, GamepadConstants.BUTTON_RB);
    public static final JoystickButton primaryLeftStickButton = new JoystickButton(primaryJoystick, GamepadConstants.BUTTON_LEFT_STICK);
    public static final JoystickButton primaryRightStickButton = new JoystickButton(primaryJoystick, GamepadConstants.BUTTON_RIGHT_STICK);
    
    //primary DPad
    public static final DPad_JoystickButton primaryDPadN = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_N_ANGLE);
    public static final DPad_JoystickButton primaryDPadNW = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_NW_ANGLE);
    public static final DPad_JoystickButton primaryDPadW = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_W_ANGLE);
    public static final DPad_JoystickButton primaryDPadSW = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_SW_ANGLE);
    public static final DPad_JoystickButton primaryDPadS = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_S_ANGLE);
    public static final DPad_JoystickButton primaryDPadSE = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_SE_ANGLE);
    public static final DPad_JoystickButton primaryDPadE = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_E_ANGLE);
    public static final DPad_JoystickButton primaryDPadNE = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_NE_ANGLE);
    
    //primary axes
    //RIGHT TRIGGER       primaryJoystick.getTriggerAxis(Hand.kRight)
    //LEFT TRIGGER        primaryJoystick.getTriggerAxis(Hand.kLeft)
    //LEFT STICK X AXIS   primaryJoystick.getX(Hand.kLeft)
    //LEFT STICK Y AXIS   primaryJoystick.getY(Hand.kLeft)
    //RIGHT STICK X AXIS  primaryJoystick.getX(Hand.kRight)
    //RIGHT STICK Y AXIS  primaryJoystick.getY(Hand.kRight)
    
    //secondary buttons
    public static final JoystickButton secondaryBackButton = new JoystickButton(secondaryJoystick, GamepadConstants.BUTTON_BACK);
    public static final JoystickButton secondaryStartButton = new JoystickButton(secondaryJoystick, GamepadConstants.BUTTON_START);
    public static final JoystickButton secondaryXButton = new JoystickButton(secondaryJoystick, GamepadConstants.BUTTON_X);
    public static final JoystickButton secondaryYButton = new JoystickButton(secondaryJoystick, GamepadConstants.BUTTON_Y);
    public static final JoystickButton secondaryBButton = new JoystickButton(secondaryJoystick, GamepadConstants.BUTTON_B);
    public static final JoystickButton secondaryAButton = new JoystickButton(secondaryJoystick, GamepadConstants.BUTTON_A);
    public static final JoystickButton secondaryLB = new JoystickButton(secondaryJoystick, GamepadConstants.BUTTON_LB);
    public static final JoystickButton secondaryRB = new JoystickButton(secondaryJoystick, GamepadConstants.BUTTON_RB);  
    public static final JoystickButton secondaryLeftStickButton = new JoystickButton(secondaryJoystick, GamepadConstants.BUTTON_LEFT_STICK);
    public static final JoystickButton secondaryRightStickButton = new JoystickButton(secondaryJoystick, GamepadConstants.BUTTON_RIGHT_STICK);
    
    //secondary DPad
    public static final DPad_JoystickButton secondaryDPadN = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_N_ANGLE);
    public static final DPad_JoystickButton secondaryDPadNW = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_NW_ANGLE);
    public static final DPad_JoystickButton secondaryDPadW = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_W_ANGLE);
    public static final DPad_JoystickButton secondaryDPadSW = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_SW_ANGLE);
    public static final DPad_JoystickButton secondaryDPadS = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_S_ANGLE);
    public static final DPad_JoystickButton secondaryDPadSE = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_SE_ANGLE);
    public static final DPad_JoystickButton secondaryDPadE = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_E_ANGLE);
    public static final DPad_JoystickButton secondaryDPadNE = new DPad_JoystickButton(secondaryJoystick, GamepadConstants.DPAD_NE_ANGLE);
    
    //secondary axes    
    //RIGHT TRIGGER       secondaryJoystick.getTriggerAxis(Hand.kRight)
    //LEFT TRIGGER        secondaryJoystick.getTriggerAxis(Hand.kLeft)
    //LEFT STICK X AXIS   secondaryJoystick.getX(Hand.kLeft)
    //LEFT STICK Y AXIS   secondaryJoystick.getY(Hand.kLeft)
    //RIGHT STICK X AXIS  secondaryJoystick.getX(Hand.kRight)
    //RIGHT STICK Y AXIS  secondaryJoystick.getY(Hand.kRight)
}
