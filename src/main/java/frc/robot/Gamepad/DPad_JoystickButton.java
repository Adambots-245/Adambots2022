/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Gamepad;

import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj2.command.button.Button;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

/**
 * A {@link Button} that gets its state from a {@link GenericHID}.
 */
public class DPad_JoystickButton extends Button {
  private final GenericHID m_joystick;
  private final int m_dpadDirection;

  /**
   * Creates a joystick button for triggering commands.
   *
   * @param joystick     The GenericHID object that has the button (e.g. Joystick, KinectStick,
   *                     etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
   */
  public DPad_JoystickButton(GenericHID joystick, int direction) {
    requireNonNullParam(joystick, "joystick", "JoystickButton");

    m_joystick = joystick;
    m_dpadDirection = direction;
  }

  /**
   * Gets the value of the joystick button.
   *
   * @return The value of the joystick button
   */
  @Override
  public boolean get() {
    int DPadValue = m_joystick.getPOV(GamepadConstants.AXIS_DPAD_POV);
    boolean isPressed = (DPadValue == m_dpadDirection);
    
    return isPressed;
  }
}