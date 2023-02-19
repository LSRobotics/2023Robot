package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;

    public VisionSubsystem() {
        super();
    }
    public void periodic() {
        // This method will be called once per scheduler run
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        
      }
    public void printValues(){
        System.out.println(tx);
        System.out.println(ty);
        System.out.println(ta);
    }
        
    
}
