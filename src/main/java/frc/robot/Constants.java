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
    public static final int fr_motor_id = 5;
    public static final int fl_motor_id = 4;
    public static final int br_motor_id = 2;
    public static final int bl_motor_id = 3;
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
    public static class TiltPID {
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
    public static final double intake_fast_speed = .75;
    public static final double intake_slow_speed = 0.25;
  }
  public static class ArmConstants {

    //TODO: Figure out all of these values
    public static final int motor_id = 32;
  }
}
