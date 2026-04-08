package frc.robot.subsystems.interfaces;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

public class LED {
    
    private static final CANdle mRGB_Candle = new CANdle(49, "CANivore");
    private static final int MAX_LED_INDEX = 42;

    public static void init() {
        CANdleConfiguration config = new CANdleConfiguration();

        config.LED.StripType = StripTypeValue.RGB;
        config.LED.BrightnessScalar = 1.0;

        mRGB_Candle.getConfigurator().apply(config);

        mRGB_Candle.setControl(new LarsonAnimation(0, MAX_LED_INDEX)
            .withColor(new RGBWColor(135, 206, 235)));
        //init animation
    }

    public static void setRainBow() {
        mRGB_Candle.setControl(new RainbowAnimation(0, MAX_LED_INDEX));
    }

    public static void setSolidGreen() {
        mRGB_Candle.setControl(new SolidColor(0, MAX_LED_INDEX)
            .withColor(new RGBWColor(0, 255, 0)));
    }

    public static void setSolidBlue() {
        mRGB_Candle.setControl(new SolidColor(0, MAX_LED_INDEX)
            .withColor(new RGBWColor(0, 0, 255)));
    }

    public static void setSolidRed() {
        mRGB_Candle.setControl(new SolidColor(0, MAX_LED_INDEX)
            .withColor(new RGBWColor(255, 0, 0)));
    }

    public static void setFlashGreen() {
        mRGB_Candle.setControl(new StrobeAnimation(0, MAX_LED_INDEX)
            .withColor(new RGBWColor(0, 255, 0)));
    }

    public static void setSolidWhite() {
        mRGB_Candle.setControl(new SolidColor(0, MAX_LED_INDEX)
            .withColor(new RGBWColor(255, 255, 255)));
    }

    public static void setFlashBlue() {
        mRGB_Candle.setControl(new StrobeAnimation(0, MAX_LED_INDEX)
            .withColor(new RGBWColor(0, 0, 255)));
    }

    public static void setFlashRed() {
        mRGB_Candle.setControl(new StrobeAnimation(0, MAX_LED_INDEX)
            .withColor(new RGBWColor(255, 0, 0)));
    }

    public static void setFlashYellow() {
        mRGB_Candle.setControl(new StrobeAnimation(0, MAX_LED_INDEX)
            .withColor(new RGBWColor(255, 255, 0)));
    }

    public static void setFlashWhite() {
        mRGB_Candle.setControl(new StrobeAnimation(0, MAX_LED_INDEX)
            .withColor(new RGBWColor(255, 255, 255)));
    }

    public static void turnOff() {
        mRGB_Candle.clearAllAnimations();
        mRGB_Candle.setControl(new SolidColor(0, MAX_LED_INDEX)
            .withColor(new RGBWColor(0, 0, 0)));
    }
}
