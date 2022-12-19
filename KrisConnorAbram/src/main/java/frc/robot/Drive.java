package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive
{
    public static void strafe(double mag, double angle)
    {
        double currentAngle1 = Map.can1.getAbsolutePosition()- Map.offset1;
        if(currentAngle1<0){currentAngle1 = currentAngle1+360;} 
        double currentAngle2 = Map.can2.getAbsolutePosition()- Map.offset2; 
        if(currentAngle2<0){currentAngle2 = currentAngle2+360;} 
        double currentAngle3 = Map.can3.getAbsolutePosition()- Map.offset3; 
        if(currentAngle3<0){currentAngle3 = currentAngle3+360;} 
        double currentAngle4 = Map.can4.getAbsolutePosition()- Map.offset4; 
        if(currentAngle4<0){currentAngle4 = currentAngle4+360;} 

        if (mag > 0.2)
        {
            Map.drive1.set(ControlMode.PercentOutput, (mag * elOptimal(angle, currentAngle1)[1])*0.2);
            Map.drive2.set(ControlMode.PercentOutput, (mag * elOptimal(angle, currentAngle2)[1])*0.2);
            Map.drive3.set(ControlMode.PercentOutput, (mag * elOptimal(angle, currentAngle3)[1])*0.2);
            Map.drive4.set(ControlMode.PercentOutput, (mag * elOptimal(angle, currentAngle4)[1])*0.2);


            Map.rotate1.set(ControlMode.PercentOutput, elOptimal(angle, currentAngle1)[0] /275);
            Map.rotate2.set(ControlMode.PercentOutput, elOptimal(angle, currentAngle2)[0] /275);
            Map.rotate3.set(ControlMode.PercentOutput, elOptimal(angle, currentAngle3)[0] /275);
            Map.rotate4.set(ControlMode.PercentOutput, elOptimal(angle, currentAngle4)[0] /275);
            
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

}
