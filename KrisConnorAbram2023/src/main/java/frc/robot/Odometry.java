package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Odometry {

    static double currentAngle1;
    static double currentAngle2;
    static double currentAngle3;
    static double currentAngle4;

    static double currentSpeed1;
    static double currentSpeed2;
    static double currentSpeed3;
    static double currentSpeed4;

    static double Speed1x;
    static double Speed1y;
    static double Speed2x;
    static double Speed2y;
    static double Speed3x;
    static double Speed3y;
    static double Speed4x;
    static double Speed4y;

    static double avgx;
    static double avgy;
    static double avgz;

    static double ang;
    static double hyp;

    public static void calcVel(double gyroAngle) {

        currentSpeed1 = Map.drive1.getSelectedSensorVelocity();
        currentSpeed2 = Map.drive2.getSelectedSensorVelocity();
        currentSpeed3 = Map.drive3.getSelectedSensorVelocity();
        currentSpeed4 = Map.drive4.getSelectedSensorVelocity();
        
        currentAngle1 = Map.can1.getAbsolutePosition() - Map.offset1;
        currentAngle2 = Map.can2.getAbsolutePosition() - Map.offset2;
        currentAngle3 = Map.can3.getAbsolutePosition() - Map.offset3;
        currentAngle4 = Map.can4.getAbsolutePosition() - Map.offset4;

        Speed1x = Math.cos(DriveTrain.toRadians(currentAngle1)) * currentSpeed1;
        Speed1y = Math.sin(DriveTrain.toRadians(currentAngle1)) * currentSpeed1;
        Speed2x = Math.cos(DriveTrain.toRadians(currentAngle2)) * currentSpeed2;
        Speed2y = Math.sin(DriveTrain.toRadians(currentAngle2)) * currentSpeed2;
        Speed3x = Math.cos(DriveTrain.toRadians(currentAngle3)) * currentSpeed3;
        Speed3y = Math.sin(DriveTrain.toRadians(currentAngle3)) * currentSpeed3;
        Speed4x = Math.cos(DriveTrain.toRadians(currentAngle4)) * currentSpeed4;
        Speed4y = Math.sin(DriveTrain.toRadians(currentAngle4)) * currentSpeed4;

        avgz = (Speed1x + Speed2x + Speed2y + Speed4y - Speed4x - Speed3x - Speed3y - Speed1y) / 4;

        currentAngle1 = Map.can1.getAbsolutePosition() - Map.offset1 - gyroAngle;
        currentAngle2 = Map.can2.getAbsolutePosition() - Map.offset2 - gyroAngle;
        currentAngle3 = Map.can3.getAbsolutePosition() - Map.offset3 - gyroAngle;
        currentAngle4 = Map.can4.getAbsolutePosition() - Map.offset4 - gyroAngle;

        Speed1x = Math.cos(DriveTrain.toRadians(currentAngle1)) * currentSpeed1;
        Speed1y = Math.sin(DriveTrain.toRadians(currentAngle1)) * currentSpeed1;
        Speed2x = Math.cos(DriveTrain.toRadians(currentAngle2)) * currentSpeed2;
        Speed2y = Math.sin(DriveTrain.toRadians(currentAngle2)) * currentSpeed2;
        Speed3x = Math.cos(DriveTrain.toRadians(currentAngle3)) * currentSpeed3;
        Speed3y = Math.sin(DriveTrain.toRadians(currentAngle3)) * currentSpeed3;
        Speed4x = Math.cos(DriveTrain.toRadians(currentAngle4)) * currentSpeed4;
        Speed4y = Math.sin(DriveTrain.toRadians(currentAngle4)) * currentSpeed4;

        avgx = (Speed1x + Speed2x + Speed3x + Speed4x) / 4;
        avgy = (Speed1y + Speed2y + Speed3y + Speed4y) / 4;

        SmartDashboard.putNumber("X vel", avgx);
        SmartDashboard.putNumber("Y vel", avgy);
        SmartDashboard.putNumber("Z vel", avgz);
    }
}
