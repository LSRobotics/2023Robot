package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

public class IntakeSubsystem extends SubsystemBase {
    
    public IntakeSubsystem() {
        super();
    }
    private WPI_TalonSRX intake1 = new WPI_TalonSRX(21);
    private WPI_TalonSRX intake2 = new WPI_TalonSRX(22);

    private CANSparkMax intakeMotor = new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless);

    private double intakePowerScalar = Constants.IntakeConstants.intake_default_speed;
    
    public void setPower(double power) {
        intakeMotor.set(power);
        //intake2.set(ControlMode.PercentOutput, -power * intakePowerScalar);
    }

    public void setPowerScalar(double scalar) {
        intakePowerScalar = scalar;
    }
        
}
