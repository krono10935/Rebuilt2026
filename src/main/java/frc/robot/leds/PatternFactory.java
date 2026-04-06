package frc.robot.leds;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

import java.util.Map;
import java.util.function.BooleanSupplier;

public class PatternFactory {

    private final static LEDPattern defaultRedPattern = createDefaultPattern(DriverStation.Alliance.Red);
    private final static LEDPattern defaultBluePattern = createDefaultPattern(DriverStation.Alliance.Blue);

    private static LEDPattern createDefaultPattern(DriverStation.Alliance alliance){
        Color primaryColor = Color.kWhite;
        Color secondaryColor;

        secondaryColor = alliance == DriverStation.Alliance.Red ?
                new Color(130, 20, 20) :
                Color.kDarkBlue;

        LEDPattern background = LEDPattern.solid(primaryColor);

        LEDPattern island = LEDPattern.steps(Map.of(0, Color.kBlack, 0.8, secondaryColor)).scrollAtRelativeSpeed(Units.Hertz.of(1)).atBrightness(Units.Percent.of(60));

        return island.overlayOn(island.reversed()).overlayOn(background);
    }

    public static LEDPattern defaultPattern(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Red ? defaultRedPattern : defaultBluePattern;
    }

    /**
     * Creates a rainbow pattern
     *
     * @return a rainbow pattern
     */
    public static LEDPattern rainbow(Frequency hz, Dimensionless brightness) {
        return LEDPattern.rainbow(120, 255).scrollAtRelativeSpeed(hz).atBrightness(brightness);
    }

    /**
     * Creates a solid pattern
     *
     * @return a solid pattern
     */
    public static LEDPattern solid(Color primaryColor, Dimensionless brightness) {
        return LEDPattern.solid(primaryColor).atBrightness(brightness);
    }

    /**
     * Creates a blinking pattern
     *
     * @return a blinking pattern
     */
    public static LEDPattern blink(Color primaryColor, Color secondaryColor, Frequency hz, Dimensionless brightness) {
        return LEDPattern.solid(primaryColor).blink(hz.asPeriod()).atBrightness(brightness)
                .overlayOn(LEDPattern.solid(secondaryColor).atBrightness(brightness));
    }

    public static LEDPattern rsl_blink(Color primaryColor, Color secondaryColor, Dimensionless brightness, BooleanSupplier rslStatus) {
        return LEDPattern.solid(primaryColor).synchronizedBlink(rslStatus).atBrightness(brightness)
                .overlayOn(LEDPattern.solid(secondaryColor).atBrightness(brightness));
    }

    public static LEDPattern skebob(Color primaryColor, Color secondaryColor, Frequency hz, Dimensionless brightness) {
        var pattern = LEDPattern.steps(Map.of(0.00, primaryColor, 0.2, Color.kBlack)).scrollAtRelativeSpeed(hz).atBrightness(brightness);

        var movingPattern = pattern.reversed();

        return movingPattern.overlayOn(pattern).overlayOn(LEDPattern.solid(secondaryColor).atBrightness(brightness));
    }

    public static LEDPattern captain_usa(Color primaryColor, Color secondaryColor, Frequency hz, Dimensionless brightness) {
        var pattern = LEDPattern.steps(Map.of(0.00, primaryColor, 0.5, secondaryColor)).scrollAtRelativeSpeed(hz).atBrightness(brightness);

        var movingPattern = pattern.breathe(hz.asPeriod());

        return movingPattern;
    }

    public static LEDPattern blue_pulse(Frequency hz) {
        var pattern = LEDPattern.steps(Map.of(0.00, Color.kGray, 0.35, Color.kWhiteSmoke, 0.7, Color.kGreen)).scrollAtRelativeSpeed(hz);
        var pattern1 = LEDPattern.steps(Map.of(0.00, Color.kDarkGray, 0.35, Color.kWhite, 0.7, Color.kDarkBlue)).scrollAtRelativeSpeed(hz);
        var breathe = pattern1.blink(hz.asPeriod());

        var patterns = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kDarkGray, Color.kWhite);

        return LEDPattern.solid(Color.kGray);

//        return breathe.overlayOn(pattern).atBrightness(brightness);
    }

    private final static LEDPattern shooterSpunUpPattern = createSpunUpPattern(true);
    private final static LEDPattern shooterNotSpunUpPattern = createSpunUpPattern(false);

    private static LEDPattern createSpunUpPattern(boolean isSpunUp){
        Color spunUpColor = isSpunUp ? Color.kGreen : Color.kRed;

        var pattern = LEDPattern.steps(Map.of(0.00, Color.kBlack, 0.85, spunUpColor)).scrollAtRelativeSpeed(Units.Hertz.of(1));

         return pattern.overlayOn(pattern.reversed());
    }

    public static LEDPattern shooterSpunUpIndicator(boolean isSpunUp){
        return isSpunUp ? shooterSpunUpPattern : shooterNotSpunUpPattern;
    }
}
