
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.*;

public class limelightSubSystem extends SubsystemBase {
    
    private NetworkTable NetworkTable;
    private int statepip;

    public limelightSubSystem() {
      NetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
      NetworkTable.getEntry("pipeline").setNumber(3);
      statepip = 0;
    }

    public void init() {
        System.out.println("init camera subsystem/n");
        statepip = 1;
    }

    @Override
    public void periodic() {
        //get Position target
        double x = NetworkTable.getEntry("tx").getDouble(0.0);
        double y = NetworkTable.getEntry("ty").getDouble(0.0);
        double area = NetworkTable.getEntry("ta").getDouble(0.0);
        double tv = NetworkTable.getEntry("tv").getDouble(0.0);

        //post to shaffelbord
        // SmartDashboard.putNumber("LimelightX", x);
        // SmartDashboard.putNumber("LimelightY", y);
        // SmartDashboard.putNumber("LimelightArea", area);
        // SmartDashboard.putNumber("Limelighttv", tv);

        SmartDashboard.putNumber("dis", util.distanceFromTarget(y));
        
    }

    //get targetX
    public double targetX() {
        double tx = NetworkTable.getEntry("tx").getDouble(0);
        return tx;
    }

    //get targetArea
    public double targetArea() {
        double area = NetworkTable.getEntry("ta").getDouble(0.0);
        return area;
    }

    //get targetY
    public double targetY(){
        return NetworkTable.getEntry("ty").getNumber(0).doubleValue();
    }

    //get targetv
    public boolean targetv() {
        boolean tv = NetworkTable.getEntry("tv").getBoolean(false);
        return tv;
    }
    
    //setpipline
    public void setpipline(int pipeline){
        pipeline = pipeline + statepip;
        NetworkTable.getEntry("pipeline").setNumber(pipeline);
        statepip = pipeline + statepip;
    }

    //LEDmode
    public void setLedMode(int mode){
        NetworkTable.getEntry("LedMode").setNumber(mode);
    }
}

