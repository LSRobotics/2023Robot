// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

    private CANSparkMax fl_motor = new CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax fr_motor = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax br_motor = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax bl_motor = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);

    private MotorControllerGroup left_motors  = new MotorControllerGroup(fl_motor, bl_motor);
    private MotorControllerGroup right_motors = new MotorControllerGroup(fr_motor, br_motor);

    private DifferentialDrive drive_controller = new DifferentialDrive(right_motors, left_motors);

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    super();

    fl_motor.setOpenLoopRampRate(0.5);
    fr_motor.setOpenLoopRampRate(0.5);
    br_motor.setOpenLoopRampRate(0.5);
    bl_motor.setOpenLoopRampRate(0.5);

    right_motors.setInverted(true);

    }

  public void drive(double driveSpeed, double turnSpeed) {
    //Drive command accessible from outside the class
    drive_controller.arcadeDrive(driveSpeed, turnSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
