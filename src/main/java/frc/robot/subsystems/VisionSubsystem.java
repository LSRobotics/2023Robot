package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {
    private double tx, ty, ta;
    private final NetworkTable m_limelightTable;
    
    
    public VisionSubsystem() {
        super();
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }
    
    
    public void periodic() {
        // This method will be called once per scheduler run
        ty = m_limelightTable.getEntry("ty").getDouble(0);
        tx = m_limelightTable.getEntry("tx").getDouble(0);
        ta = m_limelightTable.getEntry("ta").getDouble(0);
        
      }
    public void printValues(){
        System.out.println(tx);
        System.out.println(ty);
        System.out.println(ta);
    }
        
    
}
