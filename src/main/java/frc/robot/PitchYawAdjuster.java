package frc.robot;

public class PitchYawAdjuster {
    public static double GetYawFromPitch(double pitch) {
        return -1.01157 * pitch - 5.3254;
    }
}
