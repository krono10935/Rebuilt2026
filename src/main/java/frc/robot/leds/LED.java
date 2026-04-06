

package frc.robot.leds;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.leds.LEDConstants.Segments;

import java.util.HashMap;

import com.revrobotics.ColorSensorV3.LEDCurrent;

public class LED extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    private final HashMap<LEDConstants.Segments, LedSegment> segments = new HashMap<>();

    private static LED instance;

    public static LED getInstance() {
        if (instance == null) {
            instance = new LED();
        }

        return instance;
    }

    /**
     * Creates a new LED.
     */
    private LED() {
        led = new AddressableLED(LEDConstants.LED_PORT);
        led.setLength(LEDConstants.LED_COUNT_TOTAL);
        led.setColorOrder(ColorOrder.kRGB);
        buffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT_TOTAL);

        for (var segment : LEDConstants.Segments.values()) {
            var view = buffer.createView(segment.start, segment.end);
            segments.put(segment, new LedSegment(view));
        }

        setPattern(LEDConstants.Segments.ALL, PatternFactory.defaultPattern(DriverStation.Alliance.Red));
        setPattern(LEDConstants.Segments.INDEXER, PatternFactory.ballDotsPattern());
        setPattern(Segments.RIO, PatternFactory.solid(Color.kRed,Units.Percent.of(100)));
        setPattern(Segments.PDH_RIGHT, PatternFactory.solid(Color.kBlue,Units.Percent.of(100)));
        setPattern(Segments.PDH_LEFT, PatternFactory.solid(Color.kRed,Units.Percent.of(100)));
        setPattern(Segments.INDEXER, PatternFactory.solid(Color.kBlue,Units.Percent.of(100)));
         setPattern(Segments.INTAKE, PatternFactory.solid(Color.kRed,Units.Percent.of(100)));


        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic() {
        for (var segment : segments.values()) {
            segment.apply();
        }

        led.setData(buffer);
    }

    public void setPattern(LEDConstants.Segments segment, LEDPattern pattern, double timeout) {
        segments.get(segment).setPattern(pattern, timeout);
    }

    public void setPattern(LEDConstants.Segments segment, LEDPattern pattern) {
        setPattern(segment, pattern, 0);
    }

    public void clearPattern(LEDConstants.Segments segment) {
        segments.get(segment).clearPattern();
    }

    public void turnOffLed() {
        for (var segment : segments.values()) {
            segment.clearPattern();
        }
    }
}