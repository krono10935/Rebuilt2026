package frc.robot.leds;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Dimensionless;

import edu.wpi.first.wpilibj.util.Color;


/** Add your docs here. */
public class LEDConstants {
    public static final int LED_PORT = 4;
    public static final int LED_COUNT_TOTAL = 101;

    public enum Segments {
        ALL(0, LED_COUNT_TOTAL - 1),
        RIO(0, 20),
        PDH_RIGHT(21, 36),
        PDH_LEFT(37, 51),
        INDEXER(52, 68),
        INTAKE(68, 100);

        public final int start;
        public final int end;

        Segments(int start, int end) {
            if (start < 0) {
                throw new IllegalArgumentException("start index must be greater then 0");
            } else if (end < start) {
                throw new IllegalArgumentException("end index must be greater the start index");
            } else if (end >= LED_COUNT_TOTAL) {
                throw new IllegalArgumentException("end must be smaller then the buffer size");
            }
            this.start = start;
            this.end = end;
        }
    }

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