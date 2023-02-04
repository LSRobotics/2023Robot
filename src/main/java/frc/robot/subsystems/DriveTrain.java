// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class DriveTrain extends SubsystemBase {

    private CANSparkMax fl_motor = new CANSparkMax(MotorConstants.fl_motor_id, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax fr_motor = new CANSparkMax(MotorConstants.fr_motor_id, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax br_motor = new CANSparkMax(MotorConstants.br_motor_id, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax bl_motor = new CANSparkMax(MotorConstants.bl_motor_id, CANSparkMaxLowLevel.MotorType.kBrushless);

    private MotorControllerGroup left_motors  = new MotorControllerGroup(fl_motor, bl_motor);
    private MotorControllerGroup right_motors = new MotorControllerGroup(fr_motor, br_motor);

    private DifferentialDrive drive_controller = new DifferentialDrive(right_motors, left_motors);

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    super();

    //The loopRampRate could be made a constant
    fl_motor.setOpenLoopRampRate(0.5);
    fr_motor.setOpenLoopRampRate(0.5);
    br_motor.setOpenLoopRampRate(0.5);
    bl_motor.setOpenLoopRampRate(0.5);

    right_motors.setInverted(true);
    fl_motor.setIdleMode(IdleMode.kBrake);
    fr_motor.setIdleMode(IdleMode.kBrake);
    br_motor.setIdleMode(IdleMode.kBrake);
    bl_motor.setIdleMode(IdleMode.kBrake);

  }

  public void arcadeDrive(double speed, double turn) {
    drive_controller.arcadeDrive(speed, turn);
  }

  
}
