package frc.robot.commands;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class pidTurn extends PIDCommand {
    private DriveTrain driveTrain;

    public pidTurn(double angle, DriveTrain driveTrain) {
      super(new PIDController(DriveTrainConstants.TurnPID.kP, DriveTrainConstants.TurnPID.kI,
      DriveTrainConstants.TurnPID.kD), 
      driveTrain::getTurnAngle,
      driveTrain.getTurnAngle() + angle,
      (double output) -> {
        double turnSpeed = MathUtil.clamp(output, -DriveTrainConstants.TurnPID.maxSpeed, DriveTrainConstants.TurnPID.maxSpeed);
        driveTrain.arcadeDrive(0, turnSpeed);
      });
      getController().setTolerance(2,3);
      getController().enableContinuousInput(-180, 180);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        driveTrain.arcadeDrive(0,0);
    }
  }
