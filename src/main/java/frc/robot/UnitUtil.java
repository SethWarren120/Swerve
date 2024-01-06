package frc.robot;

public class UnitUtil {
    /**
     * Converts a given length or quantity in inches to the same length or quantity in meters
     * @param inches The length or quantity in inches
     * @return The length or quantity in meters
     */
    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }
    
    /**
     * Converts a given length or quantity in feet to the same length or quantity in meters
     * @param feet The length or quantity in feet
     * @return The length or quantity in meters
     */
    public static double feetToMeters(double feet) {
        return feet * 0.3048;
    }
}
