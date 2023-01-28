package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {
    static double mag = 0;
    static double angle = 0;
    static double rotation = 0;
    
    static double x;
    static double y;

    static double gyroPos;

    public static void returnToOrigin() {
        x = Map.xOdometry;
        y = Map.yOdometry;

        mag = Math.sqrt(Math.sqrt(x * x + y * y) / 10) - 0.08;
        if (mag > 0.45) {
            mag = 0.45;
        }
        angle = (Map.initialAngle - gyroPos) + 180 + (Math.atan2(y, x) / (Math.PI) * 180);
        DriveTrain.drive(mag, angle, rotation);

        SmartDashboard.putNumber("Mag", mag);
        
        gyroPos = Map.gyro.getYaw();
        Odometry.calcVel(Map.initialAngle - gyroPos);
    }
}
