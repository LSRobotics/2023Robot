package frc.robot;
import java.util.*;

//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.SerialPort;
//import com.kauailabs.navx.frc.AHRS;

//import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;


import edu.wpi.first.math.MathUtil;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
  private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();


  private double autonDriveBuffer = 0;
  private double autonTurnBuffer = 0;

  public XboxController gp1;

  DifferentialDrive myRobot;  // class that handles basic drive operations

    

    PIDController turnController;
    double rotateToAngleRate;
    
    /* The following PID Controller coefficients will need to be tuned */
    /* to match the dynamics of your drive system.  Note that the      */
    /* SmartDashboard in Test mode has support for helping you tune    */
    /* controllers by displaying a form where you can enter new P, I,  */
    /* and D constants and test the mechanism.                         */
    
    static final double kP = 0.03;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
    
    static final double kToleranceDegrees = 2.0f;    
    
    static final double kTargetAngleDegrees = 90.0f;
    
    // Channels for the wheels
    final static int leftChannel	= 0;
    final static int rightChannel	= 1;
    
   
  private PIDController movePid;
  private PIDController gyroPid;

  ComplexWidget cameraTest;
  ShuffleboardTab testTab = Shuffleboard.getTab("Test Board");

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightDrive.setInverted(true);
    initializeGamePad();
    shuffleboardStartup();
    
    autonSetDrive(0,0);
        
  
  }
  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

    driveTrain(gp1.getRightTriggerAxis()-gp1.getLeftTriggerAxis(), gp1.getLeftX());

    
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
  private void driveTrain(double power, double turn) {
    m_robotDrive.arcadeDrive(-0.8*cubicScaledDeadband(power, 0.1, 0.1),
    0.6*cubicScaledDeadband(turn, 0.1, 0.1));
    autonSetDrive(0,0);
    
    
  }            
  public double cubicScaledDeadband(double x, double deadbandCutoff, double weight){
    if (Math.abs(x) < deadbandCutoff) {
      return 0;
    } else {
      return (cubic(x, weight)- (Math.abs(x)/x) * cubic(deadbandCutoff, weight)) / (1.0 - cubic(deadbandCutoff, weight));
    }
  }
  private double cubic(double x, double w){
    return w * x * x * x  + (1.0 - w) * x;
  }
  private void initializeGamePad(){
    gp1 = new XboxController(0);
  }
  public void shuffleboardStartup(){
    /*testTab.add("camera", Camera.server.getSource())
    .withWidget(BuiltInWidgets.kCameraStream)
    .withSize( 4, 4)
    .withPosition(2 , 0);*/
  }  
  public void autonSetDrive(double drive, double turn) {
    autonDriveBuffer = drive;
    autonTurnBuffer = turn;
  }
  
}