//Pigeon Library: https://maven.ctr-electronics.com/release/com/ctre/phoenixpro/PhoenixProAnd5-frc2023-latest.json
//SparkMax LIbrary: https://software-metadata.revrobotics.com/REVLib.json

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;

public class Map {
    public static TalonFX drive1 = new TalonFX(1);
    public static TalonFX drive2 = new TalonFX(2);
    public static TalonFX drive3 = new TalonFX(3);
    public static TalonFX drive4 = new TalonFX(4);

    public static TalonFX rotate1 = new TalonFX(5);
    public static TalonFX rotate2 = new TalonFX(6);
    public static TalonFX rotate3 = new TalonFX(7);
    public static TalonFX rotate4 = new TalonFX(8);

    public static CANCoder can1 = new CANCoder(9);
    public static CANCoder can2 = new CANCoder(10);
    public static CANCoder can3 = new CANCoder(11);
    public static CANCoder can4 = new CANCoder(12);

    public static double offset1 = 116.7 + 90;
    public static double offset2 = 15.4 + 90;
    public static double offset3 = 200.1 + 90;
    public static double offset4 = 238.7 + 90;

    public static PigeonIMU gyro = new PigeonIMU(0);

    public static XboxController driver = new XboxController(0);
    public static int[] driver_mode = {0, 0, 0};

    public static double deadBand = 0.1;
    
    public static double initialAngle = 0;
    public static double straightAngle = 0;

    public static DigitalInput bumpSensor = new DigitalInput(0);
    public static Servo servoMotor = new Servo(0);

    public static Boolean LimelightLEDOn = false;
    
}

