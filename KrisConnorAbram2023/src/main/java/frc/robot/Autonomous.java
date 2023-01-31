package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {
    static double mag = 0;
    static double angle = 0;
    static double rotation = 0;
    
    static double x;
    static double y;

    static double gyroPos;

    public static double[] twistAngCoords(double angle) {
        double[] coords = {0, 0};
        coords[0] = (Math.sin(DriveTrain.toRadians(angle + 90)) + Math.cos(DriveTrain.toRadians(angle + 90)) - 1) * 1.6;
        coords[1] = (Math.sin(DriveTrain.toRadians(angle)) + Math.cos(DriveTrain.toRadians(angle)) - 1) * 1.6;

        return coords;
    }

    public static double twistCoordsAng(double[] coords) {
        angle = DriveTrain.toDegrees(Math.atan2(coords[1] + 1.6, coords[0] + 1.6));
	    if (angle < 0) {
            return angle + 360;
        }
	    return angle;
    }

    public static void returnToOrigin() {
        x = Map.xOdometry;
        y = Map.yOdometry;

        mag = Math.sqrt(Math.sqrt(x * x + y * y) / 10) - 0.08;
        if (mag > 0.45) {
            mag = 0.45;
        }
        angle = (Map.initialAngle - gyroPos) + (Math.atan2(y, x) / (Math.PI) * 180);
        rotation = twistCoordsAng(Map.change) / 720;
        DriveTrain.drive(mag, angle, rotation);

        SmartDashboard.putNumber("Mag", mag);
        
        gyroPos = Map.gyro.getYaw();
        Odometry.calcVel(Map.initialAngle - gyroPos);
    }
}
