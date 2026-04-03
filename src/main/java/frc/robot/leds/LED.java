

package frc.robot.leds;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.leds.LEDConstants.AllainceColor;

public class LED extends SubsystemBase {
    AddressableLED led;
    AddressableLEDBuffer buffer;
    StripControl controlType = StripControl.ALL;

    AddressableLEDBufferView bufferFirstSegment;
    AddressableLEDBufferView bufferSecondSegment;
    AddressableLEDBufferView bufferThirdSegment;
    AddressableLEDBufferView bufferFourthSegment;
    AddressableLEDBufferView bufferFifthSegment;
    AddressableLEDBufferView bufferSixthSegment;
    AddressableLEDBufferView bufferFrontHalf;
    AddressableLEDBufferView bufferBackHalf;

    LEDPattern defaultPattern;
    LEDPattern pattern;
    private LEDPattern[] segmentPatterns = new LEDPattern[6];

    LEDPattern closeFeederPattern;
    LEDPattern awayFeederPattern;

    boolean ledOn = true;

    public enum StripControl{
        ALL,
        HALF,
        FIRST_SEGMENT,
        SECOND_SEGMENT,
        THIRD_SEGMENT,
        FOURTH_SEGMENT,
        FIFTH_SEGMENT,
        SIXTH_SEGMENT
    }

    /** Creates a new LED. */
    public LED() {
        led = new AddressableLED(LEDConstants.LED_PORT);
        led.setLength(LEDConstants.LED_COUNT_TOTAL);
        led.setColorOrder(ColorOrder.kRGB);

        buffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT_TOTAL);

        bufferFirstSegment = buffer.createView(0,LEDConstants.FIRST_SEGMENT);

        int start = 0;

        bufferFirstSegment = buffer.createView(start, start + LEDConstants.FIRST_SEGMENT - 1);
        start += LEDConstants.FIRST_SEGMENT;

        bufferSecondSegment = buffer.createView(start, start + LEDConstants.SECOND_SEGMENT - 1);
        start += LEDConstants.SECOND_SEGMENT;

        bufferThirdSegment = buffer.createView(start, start + LEDConstants.THIRD_SEGMENT - 1);
        start += LEDConstants.THIRD_SEGMENT;

        bufferFourthSegment = buffer.createView(start, start + LEDConstants.FOURTH_SEGMENT - 1);
        start += LEDConstants.FOURTH_SEGMENT;

        bufferFifthSegment = buffer.createView(start, start + LEDConstants.FIFTH_SEGMENT - 1);
        start += LEDConstants.FIFTH_SEGMENT;

        bufferSixthSegment = buffer.createView(start, start + LEDConstants.SIXTH_SEGMENT - 1);

        bufferFrontHalf = buffer.createView(0, LEDConstants.LED_COUNT_TOTAL / 2 - 1);
        bufferBackHalf = buffer.createView(LEDConstants.LED_COUNT_TOTAL / 2, LEDConstants.LED_COUNT_TOTAL - 1);

        for (int i = 0; i < segmentPatterns.length; i++) {
            segmentPatterns[i] = LEDPattern.solid(Color.kBlack);
        }

        pattern = LEDPattern.solid(Color.kDarkGreen);
        pattern.applyTo(buffer);

        led.setData(buffer);
        led.start();
    }

    public void setLEDState(boolean isOn){
        ledOn = isOn;
    }

    @Override
    public void periodic() {

        segmentPatterns[0].applyTo(bufferFirstSegment);
        segmentPatterns[1].applyTo(bufferSecondSegment);
        segmentPatterns[2].applyTo(bufferThirdSegment);
        segmentPatterns[3].applyTo(bufferFourthSegment);
        segmentPatterns[4].applyTo(bufferFifthSegment);
        segmentPatterns[5].applyTo(bufferSixthSegment);

        if (!ledOn) {
            LEDPattern.kOff.applyTo(buffer);
        }

        led.setData(buffer);
    }

    public void setStripControl(StripControl control){
        controlType = control;
    }

    public InstantCommand setStripControlCommand(StripControl control){
        return new InstantCommand(() -> setStripControl(control));
    }


    public void setDefaultPattern(boolean isRedAlliance){

        AllainceColor allainceColor = isRedAlliance ? AllainceColor.RED : AllainceColor.BLUE;

        Color color = allainceColor.MOVING_COLOR;
        Color backgroundColor = allainceColor.BACKGORUND_COLOR;

        Dimensionless colorBrightness = allainceColor.COLOR_BRIGHTNESS;
        Dimensionless backgorundBrihtness = allainceColor.BACKGROUND_BRIGHTNESS;

        LEDPattern pattern1 = LEDPattern.steps(Map.of(0, color, 0.05, Color.kBlack))
                .scrollAtRelativeSpeed(Percent.of(LEDConstants.DEFAULT_SCROLL_SPEED).per(Second))
                .atBrightness(colorBrightness);

        LEDPattern whitePattern = LEDPattern.solid(backgroundColor).atBrightness(backgorundBrihtness);

        LEDPattern gradiant = pattern1.overlayOn(pattern1.reversed());

        defaultPattern = whitePattern;
    }

    public void putDefaultPattern(){
        pattern = defaultPattern;
    }

    public InstantCommand putDefaultPatternCommand(){
        InstantCommand command = new InstantCommand(this::putDefaultPattern);

        command.addRequirements(this);

        return command;
    }

    public void setSolidColour(Color colour){
        pattern = LEDPattern.solid(colour);
    }

    public InstantCommand SetSolidColourCommand(Color colour){
        InstantCommand command = new InstantCommand(() -> setSolidColour(colour));

        command.addRequirements(this);

        return command;
    }

    public void Blink(double seconds){
        pattern = pattern.blink(Time.ofBaseUnits(seconds, Second));
    }

    public void Blink(double onSeconds, double offSeconds){
        pattern = pattern.blink(Time.ofBaseUnits(onSeconds, Second), Time.ofBaseUnits(offSeconds, Second));
    }

    public InstantCommand BlinkCommand(double seconds){
        InstantCommand command = new InstantCommand(() -> Blink(seconds));

        command.addRequirements(this);

        return command;
    }

    public void blinkWithRSL(Color color){
        pattern = LEDPattern.solid(color).synchronizedBlink(RobotController::getRSLState);
    }

    public void blinkWithRSL(){
        pattern = LEDPattern.solid(new Color(255,20,0)).synchronizedBlink(RobotController::getRSLState);
    }

    public Command blinkWithRSLCommand(Color color){
        InstantCommand command = new InstantCommand(() -> blinkWithRSL(color));

        command.addRequirements(this);

        return command;
    }

    public InstantCommand BlinkCommand(double onSeconds, double offSeconds){
        return new InstantCommand(() -> Blink(onSeconds, offSeconds));
    }

    public void setGradient(Color... colours){
        pattern = LEDPattern.gradient(GradientType.kDiscontinuous, colours);
    }

    public void setMovingGradient(int speedPercent, Color... colours){
        pattern = LEDPattern.gradient(GradientType.kContinuous, colours).scrollAtRelativeSpeed(Percent.per(Second).of(speedPercent));
    }

    public void setRainbow(){
        pattern = LEDPattern.rainbow(255, 255);
    }

    public void setMovingRainbow(int speedPercent){
        pattern = LEDPattern.rainbow(256, 256).scrollAtRelativeSpeed(Percent.per(Second).of(speedPercent));
    }

    public void setProgressLayer(DoubleSupplier percent){
        pattern = pattern.mask(LEDPattern.progressMaskLayer(percent));
    }

    public InstantCommand setProgressLayerCommand(DoubleSupplier percent){
        return new InstantCommand(() -> setProgressLayer(percent));
    }

    public void putTestPattern(){
        segmentPatterns[0] = LEDPattern.solid(Color.kOrange);
        segmentPatterns[1] = LEDPattern.solid(Color.kBlue);
        segmentPatterns[2] = LEDPattern.solid(Color.kOrange);
        segmentPatterns[3] = LEDPattern.solid(Color.kBlue);
        segmentPatterns[4] = LEDPattern.solid(Color.kOrange);
        segmentPatterns[5] = LEDPattern.solid(Color.kBlue);
    }
}