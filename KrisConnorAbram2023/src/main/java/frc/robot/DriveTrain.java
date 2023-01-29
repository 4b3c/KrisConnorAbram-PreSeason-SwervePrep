package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class DriveTrain {

    static double[] rotateVector1 = {0, 135};
    static double[] rotateVector2 = {0, 225};
    static double[] rotateVector3 = {0, 45};
    static double[] rotateVector4 = {0, 315};

    static double[] driveVector1;
    static double[] driveVector2;
    static double[] driveVector3;
    static double[] driveVector4;

    static double currentAngle1;
    static double currentAngle2;
    static double currentAngle3;
    static double currentAngle4;

    static double[] optimal1;
    static double[] optimal2;
    static double[] optimal3;
    static double[] optimal4;

    public static void drive(double mag, double angle, double rotation) {

        double[] strafeVector = {mag, angle};

        currentAngle1 = Map.can1.getAbsolutePosition() - Map.offset1;
        currentAngle2 = Map.can2.getAbsolutePosition() - Map.offset2;
        currentAngle3 = Map.can3.getAbsolutePosition() - Map.offset3;
        currentAngle4 = Map.can4.getAbsolutePosition() - Map.offset4; 

        if (currentAngle1 < 0) {
            currentAngle1 = currentAngle1 + 360;
        }
        if (currentAngle2 < 0) {
            currentAngle2 = currentAngle2 + 360;
        }
        if (currentAngle3 < 0) {
            currentAngle3 = currentAngle3 + 360;
        }
        if (currentAngle4 < 0) {
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

        if (Math.abs(rotation) > Map.rotateDeadBand) {
            Map.straightAngle = Map.gyro.getYaw();
        }

        if (mag > Map.deadBand || Math.abs(rotation) > Map.rotateDeadBand) {
            if (mag > Map.deadBand) {
                mag = mag - Map.deadBand;
            } else if (Math.abs(rotation) > Map.rotateDeadBand && rotation > 0) {
                rotation = rotation - Map.rotateDeadBand;

            } else if (Math.abs(rotation) > Map.rotateDeadBand && rotation < 0) {
                rotation = rotation + Map.rotateDeadBand;
            }

            optimal1 = elOptimal(driveVector1[1], currentAngle1);
            optimal2 = elOptimal(driveVector2[1], currentAngle2);
            optimal3 = elOptimal(driveVector3[1], currentAngle3);
            optimal4 = elOptimal(driveVector4[1], currentAngle4);

            Map.driveDir1 = optimal1[1];
            Map.driveDir2 = optimal2[1];
            Map.driveDir3 = optimal3[1];
            Map.driveDir4 = optimal4[1];

            Map.drive1.set(ControlMode.PercentOutput, (driveVector1[0] * optimal1[1]));
            Map.drive2.set(ControlMode.PercentOutput, (driveVector2[0] * optimal2[1]));
            Map.drive3.set(ControlMode.PercentOutput, (driveVector3[0] * optimal3[1]));
            Map.drive4.set(ControlMode.PercentOutput, (driveVector4[0] * optimal4[1]));

            Map.rotate1.set(ControlMode.PercentOutput, optimal1[0] / 161.4);
            Map.rotate2.set(ControlMode.PercentOutput, optimal2[0] / 161.4);
            Map.rotate3.set(ControlMode.PercentOutput, optimal3[0] / 161.4);
            Map.rotate4.set(ControlMode.PercentOutput, optimal4[0] / 161.4);
            
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

    // Returns a double array {distance to closest target, reverse variable}
    public static double[] elOptimal(double currentAngle, double targetAngle)
    {
        double[] diffAndReverse = {0, 1};
        double diff = currentAngle - targetAngle;

        // If the target and current are within 90 degrees, just return the difference
        if (diff < 90 && diff >= -90) {
            diffAndReverse[0] = diff;
            return diffAndReverse;
        
        // If it's more than 90 but less than 270 degrees, we return the difference to the opposite angle
        } else if (diff <= 270 && diff >= -270) {
            if (diff > 0) {
                diffAndReverse[0] = diff - 180;
                diffAndReverse[1] = -1;
                return diffAndReverse;

            } else {
                diffAndReverse[0] = diff + 180;
                diffAndReverse[1] = -1;
                return diffAndReverse;
            }
        
        // If it's more than 270 degrees, 
        } else {
            if (diff > 0) {
                diffAndReverse[0] = diff - 360;
                return diffAndReverse;

            } else {
                diffAndReverse[0] = diff + 360;
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