package frc.robot.commands;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class pidDrive extends PIDCommand {
    private static PIDController drivePID = new PIDController(DriveTrainConstants.DrivePID.kP, DriveTrainConstants.DrivePID.kI,
    DriveTrainConstants.DrivePID.kP);
    private DriveTrain driveTrain;

    public pidDrive(double inches, DriveTrain driveTrain) {
        super(drivePID, 
        driveTrain::getEncoderValue,
        (driveTrain.getEncoderValue() + inches), 
        (double output) -> {
        double speed = MathUtil.clamp(output, -DriveTrainConstants.DrivePID.maxSpeed, DriveTrainConstants.DrivePID.maxSpeed);
        System.out.println(driveTrain.getEncoderValue());
        // System.out.print("speed:");
        // System.out.println(speed);
        driveTrain.arcadeDrive(speed, 0);
        },
        driveTrain);
        this.driveTrain = driveTrain;
        getController().setTolerance(8);
    }
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        driveTrain.arcadeDrive(0,0);
    }
}