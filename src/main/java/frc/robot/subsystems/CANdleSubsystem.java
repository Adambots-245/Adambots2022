// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.robot.utils.Log;

public class CANdleSubsystem extends SubsystemBase {
  private static final int LEDS_PER_ANIMATION = 30;
  private static final int LEDS_IN_STRIP = Constants.LEDS_IN_STRIP;
  private CANdle candle; //= new CANdle(Constants.CANDLE_CAN_PORT, "rio");
  private Animation toAnimate = null;

  public enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    SetAll,
    Empty
  }

  private AnimationTypes currentAnimation;
  private int red = 0;
  private int green = 0;
  private int blue = 0;
  private double vbatOutput = 0.0;
  private int candleChannel;
  private boolean animDirection = false;
  private boolean setAnim;
  private double animateSpeed = 0.5;
  private boolean clearAllAnims = false;

  public CANdleSubsystem(CANdle candleDevice) {
    this.candle = candleDevice;
    changeAnimation(AnimationTypes.SetAll);
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB; // the BTF-Lighting LED strip uses GRB format
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    candleDevice.configAllSettings(configAll, 100);
  }

  public void incrementAnimation() {
    switch (currentAnimation) {
      case ColorFlow:
        changeAnimation(AnimationTypes.Fire);
        break;
      case Fire:
        changeAnimation(AnimationTypes.Larson);
        break;
      case Larson:
        changeAnimation(AnimationTypes.Rainbow);
        break;
      case Rainbow:
        changeAnimation(AnimationTypes.RgbFade);
        break;
      case RgbFade:
        changeAnimation(AnimationTypes.SingleFade);
        break;
      case SingleFade:
        changeAnimation(AnimationTypes.Strobe);
        break;
      case Strobe:
        changeAnimation(AnimationTypes.Twinkle);
        break;
      case Twinkle:
        changeAnimation(AnimationTypes.TwinkleOff);
        break;
      case TwinkleOff:
        changeAnimation(AnimationTypes.Empty);
        break;
      case Empty:
        changeAnimation(AnimationTypes.ColorFlow);
        break;
      case SetAll:
        changeAnimation(AnimationTypes.ColorFlow);
        break;
    }
  }

  public void decrementAnimation() {
    switch (currentAnimation) {
      case ColorFlow:
        changeAnimation(AnimationTypes.TwinkleOff);
        break;
      case Fire:
        changeAnimation(AnimationTypes.ColorFlow);
        break;
      case Larson:
        changeAnimation(AnimationTypes.Fire);
        break;
      case Rainbow:
        changeAnimation(AnimationTypes.Larson);
        break;
      case RgbFade:
        changeAnimation(AnimationTypes.Rainbow);
        break;
      case SingleFade:
        changeAnimation(AnimationTypes.RgbFade);
        break;
      case Strobe:
        changeAnimation(AnimationTypes.SingleFade);
        break;
      case Twinkle:
        changeAnimation(AnimationTypes.Strobe);
        break;
      case TwinkleOff:
        changeAnimation(AnimationTypes.Twinkle);
        break;
      case Empty:
        changeAnimation(AnimationTypes.TwinkleOff);
        break;
      case SetAll:
        changeAnimation(AnimationTypes.ColorFlow);
        break;
    }
  }

  public void toggleAnimDirection() {
    animDirection = !animDirection;
  }

  public void setColors() {
    changeAnimation(AnimationTypes.SetAll);
  }

  public void setColor(Color color) {
    setColor((int) color.red, (int) color.green, (int) color.blue);
  }

  /**
   * Set the colors for the LED Strip
   * 
   * @param r - Red (0 to 255)
   * @param g - Green (0 to 255)
   * @param b - Blue (0 to 255)
   */
  public void setColor(int r, int g, int b) {
    if (r < 0 || r > 255)
      this.red = 0;
    else
      this.red = r;

    if (g < 0 || g > 255)
      this.green = 0;
    else
      this.green = g;

    if (b < 0 || b > 255)
      this.blue = 0;
    else
      this.blue = b;

    setAnim = false;
  }

  /**
   * Set an individual LED to a specified color
   * @param r
   * @param g
   * @param b
   * @param startIdx - 0 based index for starting LED. The strip starts at 8.
   * @param numOfLEDs - Number of LEDs to light up with this color
   */
  public void setLEDs(int r, int g, int b, int startIdx, int numOfLEDs){

    if (r < 0 || r > 255)
      r = 0;
    
    if (g < 0 || g > 255)
      g = 0;
    
    if (b < 0 || b > 255)
      b = 0;
    
    if (startIdx < 0 || startIdx > LEDS_IN_STRIP)
      startIdx = 0;
    
    if (numOfLEDs < 0 || numOfLEDs > LEDS_IN_STRIP)
      numOfLEDs = LEDS_IN_STRIP;

    candle.setLEDs(r, g, b, 0, startIdx, numOfLEDs);
  }

  public void setmodulateVBatOutput(double dutyCycle) {
    this.vbatOutput = dutyCycle;
  }

  /* Wrappers so we can access the CANdle from the subsystem */
  public double getVbat() {
    return candle.getBusVoltage();
  }

  public double get5V() {
    return candle.get5VRailVoltage();
  }

  public double getCurrent() {
    return candle.getCurrent();
  }

  public double getTemperature() {
    return candle.getTemperature();
  }

  public void configBrightness(double percent) {
    candle.configBrightnessScalar(percent, 0);
  }

  public void configLos(boolean disableWhenLos) {
    candle.configLOSBehavior(disableWhenLos, 0);
  }

  public void configLedType(LEDStripType type) {
    candle.configLEDType(type, 0);
  }

  public void configStatusLedBehavior(boolean offWhenActive) {
    candle.configStatusLedState(offWhenActive, 0);
  }

  public void changeAnimation(AnimationTypes toChange) {
    currentAnimation = toChange;

    switch (toChange) {
      default:
      case ColorFlow:
        candleChannel = 0;
        toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LEDS_IN_STRIP, Direction.Forward, 8);
        break;
      case Fire:
        candleChannel = 1;
        toAnimate = new FireAnimation(0.5, 0.7, LEDS_IN_STRIP, 0.8, 0.5, animDirection, 8);
        break;
      case Larson:
        candleChannel = 2;
        toAnimate = new LarsonAnimation(0, 255, 46, 0, 0.1, LEDS_IN_STRIP, BounceMode.Front, 3, 8);
        break;
      case Rainbow:
        candleChannel = 3;
        toAnimate = new RainbowAnimation(1, 0.7, LEDS_IN_STRIP, animDirection, 8);
        break;
      case RgbFade:
        candleChannel = 4;
        toAnimate = new RgbFadeAnimation(0.7, 0.4, LEDS_IN_STRIP, 8);
        break;
      case SingleFade:
        candleChannel = 5;
        toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LEDS_IN_STRIP, 8);
        break;
      case Strobe:
        candleChannel = 6;
        toAnimate = new StrobeAnimation(240, 10, 180, 0, 0.01, LEDS_IN_STRIP, 8);
        break;
      case Twinkle:
        candleChannel = 7;
        toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LEDS_IN_STRIP, TwinklePercent.Percent42, 8);
        break;
      case TwinkleOff:
        candleChannel = 8;
        toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.2, LEDS_IN_STRIP, TwinkleOffPercent.Percent76, 8);
        break;
      case Empty:
        candleChannel = 9;
        toAnimate = new RainbowAnimation(1, 0.7, LEDS_IN_STRIP, animDirection, 8);
        break;

      case SetAll:
        toAnimate = null;
        break;
    }
   
    // Log.infoF("LED Changed to %s ", currentAnimation.toString());
  }

  public void clearAllAnims() {
    clearAllAnims = true;
  }

  /**
   * Set the speed of animation - 0.0 to 1.0
   * @param speed
   */
  public void setAnimationSpeed(double speed){
    animateSpeed = speed;
  }

  @Override
  public void periodic() {

    if (candle == null) {

      // Log.info("No Candle Device Set");
      return;
    }

    // This method will be called once per scheduler run
    if (toAnimate == null) {
    
      
      if(!setAnim) {
        /* Only setLEDs once, because every set will transmit a frame */
        // candle.setLEDs(255, 255, 255, 0, 0, 1);
        // candle.setLEDs(255, 255, 0, 0, 1, 1);
        // candle.setLEDs(255, 0, 255, 0, 2, 1);
        // candle.setLEDs(255, 0, 0, 0, 3, 1);
        // candle.setLEDs(0, 255, 255, 0, 4, 1);
        // candle.setLEDs(0, 255, 0, 0, 5, 1);
        // candle.setLEDs(0, 0, 0, 0, 6, 1);
        // candle.setLEDs(0, 0, 255, 0, 7, 1);
        setAnim = true;
        // green = 255;
        candle.setLEDs((int) (red), (int) (green), (int) (blue), 0, 0, LEDS_IN_STRIP);
        //   Log.infoF("No Animation: Setting LEDs to: R(%d)G(%d)B(%d)", red, green, blue);
      }
    } else {
      toAnimate.setSpeed((animateSpeed + 1) * 0.5);
      candle.animate(toAnimate, candleChannel);
      setAnim = false;
    }

    candle.modulateVBatOutput(vbatOutput);

    if(clearAllAnims) {
      clearAllAnims = false;
      for(int i = 0; i < 10; ++i) {
          candle.clearAnimation(i);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
