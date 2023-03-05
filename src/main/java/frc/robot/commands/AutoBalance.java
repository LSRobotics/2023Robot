package frc.robot.commands;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;


public class AutoBalance extends PIDCommand {

    private DriveTrain driveTrain;

    public AutoBalance(DriveTrain driveTrain) {
      super(new PIDController(DriveTrainConstants.TiltPID.kP, DriveTrainConstants.TiltPID.kI,
      DriveTrainConstants.TiltPID.kD),
      driveTrain::getTiltAngle,
      0,
      output -> {
        double speed = MathUtil.clamp(output, -DriveTrainConstants.TiltPID.maxSpeed, DriveTrainConstants.TiltPID.maxSpeed);
        driveTrain.arcadeDrive(-speed, 0);
      },
      driveTrain
      );
      this.driveTrain = driveTrain;
      getController().setTolerance(2,4);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

    @Override
    public void initialize() {
        super.initialize();
        driveTrain.setBrake();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        driveTrain.arcadeDrive(0,0);
        driveTrain.setCoast();
    }
  }