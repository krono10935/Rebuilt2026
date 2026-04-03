

package frc.robot.leds;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.leds.LEDConstants.AllainceColor;

public class LED extends SubsystemBase {
    AddressableLED led;
    AddressableLEDBuffer buffer;

    public enum Segments {
        ALL,
        RIO,
        PDH,
        INDEXER,
        INTAKE;

        private final AddressableLEDBufferView view;

        private LEDPattern pattern = LEDPattern.kOff;

        private double expiryTimeSeconds = 0;

        Segments(int start, int end) {
            if (start < 0) {
                throw new IllegalArgumentException("start index must be greater then 0");
            } else if (end < start) {
                throw new IllegalArgumentException("end index must be greater the start index");
            } else if (end >= getInstance().buffer.getLength()) {
                throw new IllegalArgumentException("end must be smaller then the buffer size");
            }
            view = getInstance().buffer.createView(start, end);
        }

        public void setPattern(LEDPattern pattern, double timeOut) {
            this.pattern = pattern;
            this.expiryTimeSeconds = Timer.getTimestamp() + timeOut;
        }

        public void setPattern(LEDPattern pattern) {
            setPattern(pattern, 0);
        }

        public void clearPattern() {
            setPattern(LEDPattern.kOff, 0);
        }

        private void apply() {
            if (expiryTimeSeconds != 0 && Timer.getTimestamp() >= expiryTimeSeconds) {
                clearPattern();
                return;
            }

            pattern.applyTo(view);
        }
    }

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

        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic() {
        for(var segment : Segments.values()){
            segment.apply();
        }

        led.setData(buffer);
    }
}