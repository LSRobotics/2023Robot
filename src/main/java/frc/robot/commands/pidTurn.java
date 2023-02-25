package frc.robot.commands;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class pidTurn extends PIDCommand {
    private static PIDController turnPID = new PIDController(DriveTrainConstants.TurnPID.kP, DriveTrainConstants.TurnPID.kI,
    DriveTrainConstants.TurnPID.kD);
    private DriveTrain driveTrain;

    public pidTurn(double angle, DriveTrain driveTrain) {
      super(turnPID, 
      driveTrain::getTurnAngle,
      driveTrain.getTurnAngle() + angle,
      (double output) -> {
        double turnSpeed = MathUtil.clamp(output, -DriveTrainConstants.TurnPID.maxSpeed, DriveTrainConstants.TurnPID.maxSpeed);
        driveTrain.arcadeDrive(0, turnSpeed);
      });
      turnPID.enableContinuousInput(-180, 180);
    }
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        driveTrain.arcadeDrive(0,0);
    }
  }
