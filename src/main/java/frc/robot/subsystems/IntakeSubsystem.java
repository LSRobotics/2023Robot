package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class IntakeSubsystem extends SubsystemBase {
    
    public IntakeSubsystem() {}
    private static double power = 0;
    private TalonSRX talon1 = new TalonSRX(21);
    private TalonSRX talon2 = new TalonSRX(22);

    public CommandBase intakeIn() {
        //code to turn the intake on
        return this.startEnd(() -> power = 1.0, () -> power = 0.0);
    }

    public CommandBase intakeOut(){
        return this.startEnd(() -> power = -1.0, () -> power = 0.0);
    }

    public CommandBase intakeOff() {
        return this.runOnce(()-> {
            power = 0.0;
        });
    }

    public CommandBase setIntakePower() {
        //code to just set the intake to be off
        return this.run(() -> {
            //actually set power to the motors
            talon1.set(ControlMode.PercentOutput, power);
            talon2.set(ControlMode.PercentOutput, -power);
        });
    }
    


}
