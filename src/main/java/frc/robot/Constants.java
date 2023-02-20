// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class MotorConstants {
    public static final int fr_motor_id = 11;
    public static final int fl_motor_id = 13;
    public static final int br_motor_id = 12;
    public static final int bl_motor_id = 14;
  }
  public static class DriveTrainConstants {
    public static class DrivePID {
      public static final double maxSpeed = .5;
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
    }
    public static class TurnPID {
      public static final double maxSpeed = .5;
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
    }
  }
  public static class IntakeConstants {
    public static final int intake1_id = 21;
    public static final int intake2_id = 22;

    public static final double intake_motor_speed = .5;
    public static final double intake_fast_speed = 1.0;
    public static final double intake_slow_speed = 0.5;
    public static final double intake_default_speed = 0.75;
  }
  public static class ArmConstants {

    //TODO: Figure out all of these values
    public static final double minEncoderAngle = -90;
    public static final double maxEncoderAngle = 90;
    public static final double encoderUnitsPerRevolution = 4096;

    public static class UpperArm {
      public static final int motor_id = 31;
    
      public static final double kP = .1;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double speedScalar = 0.1;

    }
    public static class LowerArm {
      public static final int motor_id = 32;

      public static final double kP = .1;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double speedScalar = 0.1;


    }
  }
}
