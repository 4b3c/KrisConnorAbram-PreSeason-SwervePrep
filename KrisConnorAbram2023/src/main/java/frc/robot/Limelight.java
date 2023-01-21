package frc.robot;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Limelight
 */
public class Limelight {
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    public static boolean led = false;
    public static boolean cameraSwitched = false;

    public static double getX() {

        // Returns the X axis of the target
        double x = table.getEntry("tx").getDouble(0.0);
        return x;
    }

    public static double getY() {

        // Returns the Y axis of the target
        return Math.round((table.getEntry("ty").getDouble(0.0) / 27f)*100f)/100f; // translated degrees -27 to 27 to a value between -1 and 1
    }

    public static boolean isTarget() {
        
        // Returns true if there is a target in frame
        int tv = (int) table.getEntry("tv").getDouble(0.0);
        if (tv == 1) {
            return true;
        } else {
            return false;
        }
    }
    public static double Calibrate(double h1,double h2,double distance){
        double a1 = Math.atan(((h2-h1)/distance)) - Limelight.getY();

        return a1;
    }

    public static void stream(){

        ShuffleboardTab Driver = Shuffleboard.getTab("driver");

        HttpCamera limelight = new HttpCamera("limelight", "http://10.24.86.11:5800");
        Driver.add("Limelight", limelight).withWidget(BuiltInWidgets.kCameraStream).withPosition(2, 0).withSize(5, 3);
    }

    // turns leds on
    public static void ledOn(){

        table.getEntry("ledMode").setNumber(3);
    }

    // turns leds off
    public static void ledOff(){

        table.getEntry("ledMode").setNumber(1);
    }

    // initializes camera mode for limelight
    public static void initCamera(){

        table.getEntry("stream").setNumber(0);
        table.getEntry("camMode").setNumber(0);
    }

    // controls state of LEDs
    public static void ledControl(int button){

        if (Map.driver.getRawButtonPressed(button)){
            if (!led){

                ledOff();
                led = true;
            }
            else{

                ledOn();
                led = false;
            }
        }
    }

    // controls camera mode of limelight
    public static void cameraMode(int button){

        if (Map.driver.getRawButtonPressed(button)){
            if (!cameraSwitched){

                table.getEntry("camMode").setNumber(1);
                cameraSwitched = true;
            }
            else{

                table.getEntry("camMode").setNumber(0);
                cameraSwitched = false;
            }

        }
    }
}