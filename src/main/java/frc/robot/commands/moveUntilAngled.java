package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class moveUntilAngled extends CommandBase {

    private static final double speed = 0.42;
    private static final double angleThreshold = 17;

    private int direction; 
    private double startingAngle;
    private DriveTrain driveTrain;

    //direction must be 1 or -1
    public moveUntilAngled(DriveTrain driveTrain, int direction) {
        startingAngle = driveTrain.getTiltAngle();
        this.driveTrain = driveTrain;
        this.direction = direction;
    }

    @Override
    public void execute() {
        super.execute();
        driveTrain.arcadeDrive(speed * direction, 0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs((driveTrain.getTiltAngle() - startingAngle)) > angleThreshold;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.arcadeDrive(0, 0);
    }
}
