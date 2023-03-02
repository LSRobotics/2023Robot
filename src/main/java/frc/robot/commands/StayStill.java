package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class StayStill extends CommandBase {
    DriveTrain driveTrain;
    public StayStill(DriveTrain driveTrain) {
        super();
        addRequirements(driveTrain);

        this.driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        driveTrain.setBrake();
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
        driveTrain.setCoast();
    }
}
