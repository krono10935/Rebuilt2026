package frc.robot.leds;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.intake.Intake;

import javax.swing.*;
import java.util.regex.Pattern;

/** Add your docs here. */
public class LEDConstants {
    public static final int LED_PORT = 4;

    public static final int FIRST_SEGMENT = 12;
    public static final int SECOND_SEGMENT = 8;
    public static final int THIRD_SEGMENT = 14;
    public static final int FOURTH_SEGMENT = 15;
    public static final int FIFTH_SEGMENT = 17;
    public static final int SIXTH_SEGMENT = 29;

    public static final int LED_COUNT_TOTAL = FIRST_SEGMENT + SECOND_SEGMENT + THIRD_SEGMENT + FOURTH_SEGMENT + FIFTH_SEGMENT + SIXTH_SEGMENT;

    public static LedPattern pattern;

    public static final double DEFAULT_SCROLL_SPEED = 20;

    public enum AllainceColor{
        BLUE(Color.kWhite, Color.kMidnightBlue, 60, 100),
        RED(Color.kWhite, new Color(130,20,20),60, 100);

        public final Color MOVING_COLOR;
        public final Color BACKGORUND_COLOR;

        public final Dimensionless COLOR_BRIGHTNESS;
        public final Dimensionless BACKGROUND_BRIGHTNESS;

        AllainceColor(Color color, Color backGround, int colorBrightness, int backgorundBrihtness){
            MOVING_COLOR = color;
            BACKGORUND_COLOR = backGround;
            COLOR_BRIGHTNESS = Percent.of(colorBrightness);
            BACKGROUND_BRIGHTNESS = Percent.of(backgorundBrihtness);
        }
    }


}