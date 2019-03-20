package frc.robot;

public class PitchYawAdjuster {
    public static double GetYawFromPitch(double pitch) {
        return -1.02685 * pitch - 5.48513;//-1.01707 * pitch + -5.39406;//-1.01157 * pitch - 5.3254;
    }
}
