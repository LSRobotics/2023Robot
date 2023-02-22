package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    public IntakeSubsystem() {
        super();
    }

    private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(15);

    private double intakePowerScalar = IntakeConstants.intake_default_speed;
    
    public void setPower(double power) {
        intakeMotor.set(power);
    } 

    public void setPowerScalar(double scalar) {
        intakePowerScalar = scalar;
    }
        
}
