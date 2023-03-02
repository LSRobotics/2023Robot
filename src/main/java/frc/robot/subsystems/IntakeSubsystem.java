package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class IntakeSubsystem extends SubsystemBase {
    
    private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(IntakeConstants.motor_id);
    public IntakeSubsystem() {
        super();
        intakeMotor.setInverted(true);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
    }
    
    public void setPower(double power) {
        intakeMotor.set(power);
    }
        
}
