package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
public static NetworkTableEntry tx = table.getEntry("tx");
public static double xOffset = tx.getDouble(0.0);
    public static void track (){
       

NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry ta = table.getEntry("ta");

//read values periodically
double x = tx.getDouble(0.0);
double y = ty.getDouble(0.0);
double area = ta.getDouble(0.0);

//post to smart dashboard periodically
SmartDashboard.putNumber("LimelightX", x);
SmartDashboard.putNumber("LimelightY", y);
SmartDashboard.putNumber("LimelightArea", area);

}
 public static void setPipeline(int pipeline){
    NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
    pipelineEntry.setNumber(pipeline);

    }
}


