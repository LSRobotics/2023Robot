package frc.robot.commands;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;


public class AutoBalance extends PIDCommand {
    private static PIDController tiltPID = new PIDController(DriveTrainConstants.TiltPID.kP, DriveTrainConstants.TiltPID.kI,
    DriveTrainConstants.TiltPID.kD);

    private DriveTrain driveTrain;

    public AutoBalance(DriveTrain driveTrain) {
      super(tiltPID,
      () -> {return driveTrain.getTiltAngle();},
      0,
      (double output) -> {
        double speed = MathUtil.clamp(output, -DriveTrainConstants.TiltPID.maxSpeed, DriveTrainConstants.TiltPID.maxSpeed);
        driveTrain.arcadeDrive(speed, 0);
      });

      this.driveTrain = driveTrain;
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        driveTrain.arcadeDrive(0,0);
    }
  }