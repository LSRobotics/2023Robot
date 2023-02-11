package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    
    public IntakeSubsystem() {
        super();
    }
    private WPI_TalonSRX intake1 = new WPI_TalonSRX(Constants.IntakeConstants.intake1_id);
    private WPI_TalonSRX intake2 = new WPI_TalonSRX(Constants.IntakeConstants.intake2_id);
    private double intakePowerScalar = Constants.IntakeConstants.intake_default_speed;
    
    public void setPower(double power) {
        intake1.set(ControlMode.PercentOutput, power * intakePowerScalar);
        intake2.set(ControlMode.PercentOutput, -power * intakePowerScalar);
    }

    public void setPowerScalar(double scalar) {
        intakePowerScalar = scalar;
    }
        
}
