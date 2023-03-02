// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
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
  private double driveSpeed = 0;
  private double turnSpeed = 0;
  
  //please check navx port
  private AHRS navx = new AHRS(SerialPort.Port.kMXP);

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    super();
    navx.calibrate();
    fl_motor.setSelectedSensorPosition(0);
    right_motors.setInverted(true);
    // set the controller to understand continuous angle input
  }

  @Override
  public void periodic() {
      // TODO Auto-generated method stub
      super.periodic();
      drive_controller.arcadeDrive(driveSpeed, turnSpeed);
  }

  public void arcadeDrive(double speed, double rotation) {
    driveSpeed = speed;
    turnSpeed = rotation;
  }

  public double getEncoderValue() {
    // Get the average value of all encoders
    double total = 0;
    total += fl_motor.getSelectedSensorPosition();
    //total += bl_motor.getSelectedSensorPosition();
    //total -= fr_motor.getSelectedSensorPosition();
    //total -= br_motor.getSelectedSensorPosition();
    return (total * DriveTrainConstants.encoderValueToInches);
  }

  public void setBrake() {
    fr_motor.setNeutralMode(NeutralMode.Brake);
    fl_motor.setNeutralMode(NeutralMode.Brake);
    bl_motor.setNeutralMode(NeutralMode.Brake);
    br_motor.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoast() {
    fr_motor.setNeutralMode(NeutralMode.Coast);
    fl_motor.setNeutralMode(NeutralMode.Coast);
    bl_motor.setNeutralMode(NeutralMode.Coast);
    br_motor.setNeutralMode(NeutralMode.Coast);
  }

  public double getTurnAngle() {
    return navx.getAngle();
  }

  public double getTiltAngle() {
    return navx.getRoll();
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
      drive_controller.arcadeDrive(driveSpeed.getAsDouble(), turnSpeed.getAsDouble());
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      drive_controller.arcadeDrive(0, 0);
    }
  }
  

}
