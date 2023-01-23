package frc.robot;

public class Autonomous {
    static double mag = 0;
    static double angle = 0;
    static double rotation = 0;
    
    static double x;
    static double y;

    public static void returnToOrigin() {
        x = Map.xOdometry;
        y = Map.yOdometry;

        mag = Math.sqrt(x * x + y * y);
        angle = 180 + (Math.atan2(y, -x) / (Math.PI) * 180);
        DriveTrain.drive(mag / 300000, angle, rotation);
    }
}
