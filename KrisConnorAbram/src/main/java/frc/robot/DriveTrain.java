package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain {

    static double[] rotateVector1 = {0, 45 + 90};
    static double[] rotateVector2 = {0, 45 + 180};
    static double[] rotateVector3 = {0, 45};
    static double[] rotateVector4 = {0, 45 + 270};

    static double[] driveVector1;
    static double[] driveVector2;
    static double[] driveVector3;
    static double[] driveVector4;

    static double currentAngle1;
    static double currentAngle2;
    static double currentAngle3;
    static double currentAngle4;

    public static void drive(double mag, double angle, double rotation) {

        double[] strafeVector = {mag, angle};

        currentAngle1 = Map.can1.getAbsolutePosition() - Map.offset1;
        currentAngle2 = Map.can2.getAbsolutePosition() - Map.offset2;
        currentAngle3 = Map.can3.getAbsolutePosition() - Map.offset3;
        currentAngle4 = Map.can4.getAbsolutePosition() - Map.offset4; 

        if (currentAngle1<0) {
            currentAngle1 = currentAngle1 + 360;
        }

        if (currentAngle2<0) {
            currentAngle2 = currentAngle2 + 360;
        }

        if (currentAngle3<0) {
            currentAngle3 = currentAngle3 + 360;
        }

        if (currentAngle4<0) {
            currentAngle4 = currentAngle4 + 360;
        }

        rotateVector1[0] = rotation;
        rotateVector2[0] = rotation;
        rotateVector3[0] = rotation;
        rotateVector4[0] = rotation;

        driveVector1 = addArray(strafeVector, rotateVector1);
        driveVector2 = addArray(strafeVector, rotateVector2);
        driveVector3 = addArray(strafeVector, rotateVector3);
        driveVector4 = addArray(strafeVector, rotateVector4);

        SmartDashboard.putNumber("can1pos", Map.can1.getAbsolutePosition());
        SmartDashboard.putNumber("can2pos", Map.can2.getAbsolutePosition());
        SmartDashboard.putNumber("can3pos", Map.can3.getAbsolutePosition());
        SmartDashboard.putNumber("can4pos", Map.can4.getAbsolutePosition());

        SmartDashboard.putNumber("target1angle", angle + Map.offset1);

        if (Math.abs(rotation) > Map.deadBand) {
            Map.straightAngle = Map.gyro.getYaw();
        }

        if (mag > Map.deadBand || Math.abs(rotation) > Map.deadBand) {
            mag = mag - Map.deadBand;

            if (rotation > 0) {
                rotation = rotation - Map.deadBand;

            } else {
                rotation = rotation + Map.deadBand;
            }

            Map.drive1.set(ControlMode.PercentOutput, (driveVector1[0] * elOptimal(driveVector1[1], currentAngle1)[1])*0.5);
            Map.drive2.set(ControlMode.PercentOutput, (driveVector2[0] * elOptimal(driveVector2[1], currentAngle2)[1])*0.5);
            Map.drive3.set(ControlMode.PercentOutput, (driveVector3[0] * elOptimal(driveVector3[1], currentAngle3)[1])*0.5);
            Map.drive4.set(ControlMode.PercentOutput, (driveVector4[0] * elOptimal(driveVector4[1], currentAngle4)[1])*0.5);

            Map.rotate1.set(ControlMode.PercentOutput, elOptimal(driveVector1[1], currentAngle1)[0] / 168.4);
            Map.rotate2.set(ControlMode.PercentOutput, elOptimal(driveVector2[1], currentAngle2)[0] / 168.4);
            Map.rotate3.set(ControlMode.PercentOutput, elOptimal(driveVector3[1], currentAngle3)[0] / 168.4);
            Map.rotate4.set(ControlMode.PercentOutput, elOptimal(driveVector4[1], currentAngle4)[0] / 168.4);
            
        } else {
            Map.drive1.set(ControlMode.PercentOutput, 0);
            Map.drive2.set(ControlMode.PercentOutput, 0);
            Map.drive3.set(ControlMode.PercentOutput, 0);
            Map.drive4.set(ControlMode.PercentOutput, 0);

            Map.rotate1.set(ControlMode.PercentOutput, 0);
            Map.rotate2.set(ControlMode.PercentOutput, 0);
            Map.rotate3.set(ControlMode.PercentOutput, 0);
            Map.rotate4.set(ControlMode.PercentOutput, 0);

        }

    }

    public static double[] elOptimal(double currentAngle, double targetAngle)
    {
        double[] diffAndReverse = {0, 1};

        double diff = currentAngle - targetAngle;
        double posDiff = Math.abs(diff);

        if (posDiff < 90) {
            diffAndReverse[0] = diff;

            return diffAndReverse;
        
        } else if (posDiff < 270) {
            if (diff > 0) {
                diffAndReverse[0] = diff - 180;
                diffAndReverse[1] = -1;

                return diffAndReverse;

            } else {
                diffAndReverse[0] = diff + 180;
                diffAndReverse[1] = -1;

                return diffAndReverse;
            }
        
        } else {
            if (diff > 0) {
                diffAndReverse[0] = -diff - 360;

                return diffAndReverse;

            } else {
                diffAndReverse[0] = -diff + 360;

                return diffAndReverse;
            }
            
        }

    }

    public static double[] addArray(double[] arr1, double[] arr2) {
        double magnitudeOne = arr1[0];
        double angleOne = arr1[1];

        double magnitudeTwo = arr2[0];
        double angleTwo = arr2[1];

        double newX = (magnitudeOne * Math.cos(toRadians(angleOne))) + (magnitudeTwo * Math.cos(toRadians(angleTwo)));
        double newY = (magnitudeOne * Math.sin(toRadians(angleOne))) + (magnitudeTwo * Math.sin(toRadians(angleTwo)));

        double newAngle = toDegrees(Math.atan2(newY, newX));
        double newMag = Math.sqrt((newX*newX) + (newY * newY));

        double[] resultingVector = {newMag, newAngle};

        return resultingVector;
    }
    
    public static double toRadians(double angle) {
        return (angle * Math.PI) / 180;
    }

    public static double toDegrees(double angle) {
        return (angle * 180) / Math.PI;
    }

}
