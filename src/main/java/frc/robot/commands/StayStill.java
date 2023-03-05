package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LEDSubsystem;

public class StayStill extends CommandBase {
    DriveTrain driveTrain;
    LEDSubsystem ledSubsystem;
    public StayStill(DriveTrain driveTrain, LEDSubsystem ledSubsystem) {
        super();
        addRequirements(driveTrain);

        this.driveTrain = driveTrain;
        this.ledSubsystem = ledSubsystem;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        driveTrain.setBrake();
        ledSubsystem.setBrakeMode();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        driveTrain.arcadeDrive(0,0);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        ledSubsystem.disableBrakeMode();
        driveTrain.setCoast();
    }
}
