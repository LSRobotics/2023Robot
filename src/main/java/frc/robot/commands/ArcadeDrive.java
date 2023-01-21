package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class ArcadeDrive extends CommandBase {
    
    private final DriveTrain m_DriveTrain;

    //These need to be lambdas (DoubleSupplier functions) because they need to be called every loop, returning different values each time
    private final DoubleSupplier driveSpeed;
    private final DoubleSupplier turnSpeed;

    public ArcadeDrive(DoubleSupplier speed, DoubleSupplier turn, DriveTrain driveTrain) {
        m_DriveTrain = driveTrain;
        driveSpeed = speed;
        turnSpeed = turn;
        addRequirements(m_DriveTrain);
    }

    
    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        m_DriveTrain.drive(driveSpeed.getAsDouble(), turnSpeed.getAsDouble());
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        m_DriveTrain.drive(0, 0);
    }
}
