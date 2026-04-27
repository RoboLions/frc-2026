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
    private static final int MAX_LED_INDEX = 68;

    public static void init() {
        CANdleConfiguration config = new CANdleConfiguration();

        config.LED.StripType = StripTypeValue.RGB;
        config.LED.BrightnessScalar = 0.25;

        mRGB_Candle.getConfigurator().apply(config);
    }

    public static void setRainBow() {
        mRGB_Candle.setControl(new RainbowAnimation(0, MAX_LED_INDEX)
            .withFrameRate(100)
            .withBrightness(0.25));
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
            .withColor(new RGBWColor(0, 0, 255))
            .withFrameRate(20));
    }

    public static void setFlashRed() {
        mRGB_Candle.setControl(new StrobeAnimation(0, MAX_LED_INDEX)
            .withColor(new RGBWColor(255, 0, 0))
            .withFrameRate(40));
    }

    public static void setFlashYellow() {
        mRGB_Candle.setControl(new StrobeAnimation(0, MAX_LED_INDEX)
            .withColor(new RGBWColor(255, 255, 0))
            .withFrameRate(10));
    }

    public static void setFlashWhite() {
        mRGB_Candle.setControl(new StrobeAnimation(0, MAX_LED_INDEX)
            .withColor(new RGBWColor(255, 255, 255))
            .withFrameRate(20));
    }

    public static void setFlashPurple() {
        mRGB_Candle.setControl(new StrobeAnimation(0, MAX_LED_INDEX)
            .withColor(new RGBWColor(255, 0, 255))
            .withFrameRate(5));
    }

    public static void setLarsonWhite() {
        mRGB_Candle.setControl(new LarsonAnimation(0, MAX_LED_INDEX)
            .withColor(new RGBWColor(255, 255, 255))
            .withSize(15)
            .withFrameRate(50));
    }

    public static void turnOff() {
        mRGB_Candle.clearAllAnimations();
        mRGB_Candle.setControl(new SolidColor(0, MAX_LED_INDEX)
            .withColor(new RGBWColor(0, 0, 0)));
    }
}
