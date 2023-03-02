package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;



/** An example command that uses an example subsystem. */
public class VisionAlign extends PIDCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveTrain driveTrain;

  public VisionAlign(DriveTrain DriveTrainSubsystem, VisionSubsystem VisionSubsystem) {
    super(
      new PIDController(2, 0, .3),
      VisionSubsystem::getTx,
      0,
      output -> {
        double speed = MathUtil.clamp(output, -.4, .4);
        DriveTrainSubsystem.arcadeDrive(0, -speed);
      },
      DriveTrainSubsystem,
      VisionSubsystem
    );
    driveTrain = DriveTrainSubsystem;
    getController().setTolerance(2.5);

  }

  public double getError() {
    return getController().getPositionError();
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