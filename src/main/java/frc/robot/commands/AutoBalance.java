package frc.robot.commands;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.PIDCommand;


public class AutoBalance extends PIDCommand {

    private DriveTrain driveTrain;
    private boolean balanceState;
    private Timer balanceTimer = new Timer();
    private final double waitTime = 0.2;

    public AutoBalance(DriveTrain driveTrain) {
      super(new PIDController(DriveTrainConstants.TiltPID.kP, DriveTrainConstants.TiltPID.kI,
      DriveTrainConstants.TiltPID.kD),
      driveTrain::getTiltAngle,
      0,
      output -> {
        double speed = MathUtil.clamp(output, -1, 1) * DriveTrainConstants.TiltPID.maxSpeed;
        driveTrain.arcadeDrive(-speed, 0);
      },
      driveTrain
      );
      this.driveTrain = driveTrain;
      getController().setTolerance(3,0.5);
    }

    @Override
    public boolean isFinished() {
        final boolean lastBalanceState = balanceState;
        final boolean currentBalanceState = getController().atSetpoint();
        balanceState = currentBalanceState;
        System.out.println(currentBalanceState);
        System.out.println("Velocity and Position error respectively:");
        System.out.println(getController().getVelocityError());
        System.out.println(getController().getPositionError());

        if (currentBalanceState)
        {
          balanceTimer.start();
        }
        else if (!currentBalanceState && balanceTimer.get() > 0) {
          balanceTimer.reset();
          balanceTimer.stop();
        }

        if(balanceTimer.get() > 0) {
          System.out.println(balanceTimer.get());
        }

        return balanceTimer.hasElapsed(waitTime);
    }

    @Override
    public void initialize() {
        super.initialize();
        driveTrain.setBrake();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        driveTrain.arcadeDrive(0,0);
        driveTrain.setCoast();
    }
  }