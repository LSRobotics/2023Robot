package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkMaxLowLevel;

public class IntakeSubsystem extends SubsystemBase {
    
    public IntakeSubsystem() {
        super();
    }

    private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.motor_id, CANSparkMaxLowLevel.MotorType.kBrushless);

    private double intakePowerScalar = IntakeConstants.intake_default_speed;
    
    public void setPower(double power) {
        intakeMotor.set(power);
    }

    public void setPowerScalar(double scalar) {
        intakePowerScalar = scalar;
    }
        
}
