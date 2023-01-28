package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.swing.text.html.HTML.Tag;
import javax.swing.text.html.parser.TagElement;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
public static NetworkTableEntry tx = table.getEntry("tx");
public static double xOffset = tx.getDouble(0.0);
public static boolean pipelineOneOn = false; 
   
    // change pid vlaues here
    
    public static PIDController vPid = new PIDController(40, 0, 0);
   
 public static void pipelineOne(){
    NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
    pipelineEntry.setNumber(1);

    }
    public static void pipelineZero(){
        NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
        pipelineEntry.setNumber(0);
    
        }


    public static void track (){

    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tagEntry = table.getEntry("tagEntry");
    
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double tagID = tagEntry.getInteger(0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

        SmartDashboard.putNumber("Targeted Tag", tagID );
    }

}
}


