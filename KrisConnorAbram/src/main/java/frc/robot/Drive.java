package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive
{
    public static void strafe(double mag, double angle, double rotation)
    {
        double currentAngle1 = Map.can1.getAbsolutePosition()- Map.offset1;
        if(currentAngle1<0){currentAngle1 = currentAngle1+360;} 
        double currentAngle2 = Map.can2.getAbsolutePosition()- Map.offset2; 
        if(currentAngle2<0){currentAngle2 = currentAngle2+360;} 
        double currentAngle3 = Map.can3.getAbsolutePosition()- Map.offset3; 
        if(currentAngle3<0){currentAngle3 = currentAngle3+360;} 
        double currentAngle4 = Map.can4.getAbsolutePosition()- Map.offset4; 
        if(currentAngle4<0){currentAngle4 = currentAngle4+360;}

        double[] rotateVector1 = {rotation, 45 + 90};
        double[] rotateVector2 = {rotation, 45 + 180};
        double[] rotateVector3 = {rotation, 45};
        double[] rotateVector4 = {rotation, 45 + 270};
        double[] strafeVector = {mag, angle};
        double[] driveVector1 = addArray(strafeVector, rotateVector1);
        double[] driveVector2 = addArray(strafeVector, rotateVector2);
        double[] driveVector3 = addArray(strafeVector, rotateVector3);
        double[] driveVector4 = addArray(strafeVector, rotateVector4);
        

        if (mag > 0.2 || Math.abs(rotation) > 0.2)
        {
            Map.drive1.set(ControlMode.PercentOutput, (driveVector1[0] * elOptimal(driveVector1[1], currentAngle1)[1])*0.5);
            Map.drive2.set(ControlMode.PercentOutput, (driveVector2[0] * elOptimal(driveVector2[1], currentAngle2)[1])*0.5);
            Map.drive3.set(ControlMode.PercentOutput, (driveVector3[0] * elOptimal(driveVector3[1], currentAngle3)[1])*0.5);
            Map.drive4.set(ControlMode.PercentOutput, (driveVector4[0] * elOptimal(driveVector4[1], currentAngle4)[1])*0.5);


            Map.rotate1.set(ControlMode.PercentOutput, elOptimal(driveVector1[1], currentAngle1)[0] /275);
            Map.rotate2.set(ControlMode.PercentOutput, elOptimal(driveVector2[1], currentAngle2)[0] /275);
            Map.rotate3.set(ControlMode.PercentOutput, elOptimal(driveVector3[1], currentAngle3)[0] /275);
            Map.rotate4.set(ControlMode.PercentOutput, elOptimal(driveVector4[1], currentAngle4)[0] /275);
            
        }
           
        else
        {
            Map.drive1.set(ControlMode.PercentOutput, 0);
            Map.drive2.set(ControlMode.PercentOutput, 0);
            Map.drive3.set(ControlMode.PercentOutput, 0);
            Map.drive4.set(ControlMode.PercentOutput, 0);


            Map.rotate1.set(ControlMode.PercentOutput, 0);
            Map.rotate2.set(ControlMode.PercentOutput, 0);
            Map.rotate3.set(ControlMode.PercentOutput, 0);
            Map.rotate4.set(ControlMode.PercentOutput, 0);

        }

        SmartDashboard.putNumber("currentAngle", currentAngle1);
        SmartDashboard.putNumber("targetAngle", angle);
        SmartDashboard.putNumber("rotation %", (currentAngle1 - angle) / 1000);

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
                diffAndReverse[0] = diff - 360;
                return diffAndReverse;
            } else {
                diffAndReverse[0] = diff + 360;
                return diffAndReverse;
            }
            
        }

    }

    public static double[] addArray(double[] arr1, double[] arr2) {
        double[] resultingVector = {0, 0};
        double magnitudeOne = arr1[0];
        double angleOne = arr1[1];
        double magnitudeTwo = arr2[0];
        double angleTwo = arr2[1];

        double newX = (magnitudeOne * Math.cos(toRadians(angleOne))) + (magnitudeTwo * Math.cos(toRadians(angleTwo)));
        double newY = (magnitudeOne * Math.sin(toRadians(angleOne))) + (magnitudeTwo * Math.sin(toRadians(angleTwo)));

        double newAngle = toDegrees(Math.atan2(newY, newX));
        double newMag = Math.sqrt((newX*newX) + (newY*newY));

        resultingVector[0] = newMag;
        resultingVector[1] = newAngle;

        return resultingVector;
    }
    
    public static double toRadians(double angle) {
        return (angle * Math.PI) / 180;
    }

    public static double toDegrees(double angle) {
        return (angle * 180) / Math.PI;
    }

}
