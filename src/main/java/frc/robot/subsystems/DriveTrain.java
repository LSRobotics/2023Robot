// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrain extends SubsystemBase {

  private WPI_TalonSRX fl_motor = new WPI_TalonSRX(MotorConstants.fl_motor_id);
  private WPI_TalonSRX fr_motor = new WPI_TalonSRX(MotorConstants.fr_motor_id);
  private WPI_TalonSRX br_motor = new WPI_TalonSRX(MotorConstants.br_motor_id);
  private WPI_TalonSRX bl_motor = new WPI_TalonSRX(MotorConstants.bl_motor_id);

  private MotorControllerGroup left_motors = new MotorControllerGroup(fl_motor, bl_motor);
  private MotorControllerGroup right_motors = new MotorControllerGroup(fr_motor, br_motor);

  private DifferentialDrive drive_controller = new DifferentialDrive(right_motors, left_motors);

  private PIDController drivePID = new PIDController(DriveTrainConstants.DrivePID.kP, DriveTrainConstants.DrivePID.kI,
      DriveTrainConstants.DrivePID.kP);
  private PIDController turnPID = new PIDController(DriveTrainConstants.TurnPID.kP, DriveTrainConstants.TurnPID.kI,
      DriveTrainConstants.TurnPID.kD);
  private PIDController tiltPID = new PIDController(DriveTrainConstants.TiltPID.kP, DriveTrainConstants.TiltPID.kI,
      DriveTrainConstants.TiltPID.kD);

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    super();

    right_motors.setInverted(true);
    // set the controller to understand continuous angle input
    turnPID.enableContinuousInput(-180, 180);
  }

  private double getEncoderValue() {
    // NOTE: IMPLEMENT THIS
    return 0;
  }

  private double getTurnAngle() {
    // NOTE: IMPLEMENT THIS
    return 0;
  }

  private double getTiltAngle() {
    // NOTE: IMPLEMENT THIS
    return 0;
  }


  public class pidDrive extends PIDCommand {
    public pidDrive(double distance) {
      super(drivePID, 
      () -> {return getEncoderValue();},
      distance, 
      (double output) -> {
        double speed = MathUtil.clamp(output, -DriveTrainConstants.DrivePID.maxSpeed, DriveTrainConstants.DrivePID.maxSpeed);
        drive_controller.arcadeDrive(speed, 0);
      });
    }
  }

  public class pidTurn extends PIDCommand {
    public pidTurn(double angle) {
      super(turnPID, 
      () -> {return getTurnAngle();},
      angle,
      (double output) -> {
        double turnSpeed = MathUtil.clamp(output, -DriveTrainConstants.TurnPID.maxSpeed, DriveTrainConstants.TurnPID.maxSpeed);
        drive_controller.arcadeDrive(0, turnSpeed);
      });
    }
  }

  public class autoBalance extends PIDCommand {
    public autoBalance() {
      super(tiltPID,
      () -> {return getTiltAngle();},
      0,
      (double output) -> {
        double speed = MathUtil.clamp(output, -DriveTrainConstants.TiltPID.maxSpeed, DriveTrainConstants.TiltPID.maxSpeed);
        drive_controller.arcadeDrive(speed, 0);
      });
    }
  }

  // This is the command for handling ArcadeDrive logic externally. It is located
  // within this subsystem because only this subsystem accesses it.
  public class ArcadeDriveCommand extends CommandBase {

    // These need to be lambdas (DoubleSupplier functions) because they need to be
    // called every loop, returning different values each time
    private final DoubleSupplier driveSpeed;
    private final DoubleSupplier turnSpeed;

    public ArcadeDriveCommand(DoubleSupplier speed, DoubleSupplier turn) {
      driveSpeed = speed;
      turnSpeed = turn;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
      // This is only able to directly access drive_controller becuase it is a nested
      // class
      // NOTE: if this code is changed to no longer be a nested class (for accessing
      // another subsystem or something), this will no longer be able to directly
      // access the drive (you just need to pass the subsystem and write an accessor
      // method then)
      drive_controller.arcadeDrive(driveSpeed.getAsDouble() * .7, turnSpeed.getAsDouble() * .5);
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      drive_controller.arcadeDrive(0, 0);
    }
  }
  

}
