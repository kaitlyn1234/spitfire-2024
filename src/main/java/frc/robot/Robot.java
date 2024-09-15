// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.DigitalInput;
//import com.revrobotics.blinkin;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	
public class Blinkin {

  Joystick driverController = new Joystick(0);

public void teleopPeriodic() {
          // Retrieve Limelight values
          double tx = limelightTable.getEntry("tx").getDouble(0.0); // Horizontal offset from crosshair to target
          double ty = limelightTable.getEntry("ty").getDouble(0.0); // Vertical offset from crosshair to target
          double ta = limelightTable.getEntry("ta").getDouble(0.0); // Target area (percentage of image)
          double tv = limelightTable.getEntry("tv").getDouble(0.0); // Target validity (1 if target detected, 0 otherwise)
  
          // Retrieve AprilTag values
          double tid = limelightTable.getEntry("tid").getDouble(-1); // AprilTag ID, -1 if no tag is detected
          double[] defaultPose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Default values if no pose data is found
          double[] botPose = limelightTable.getEntry("botpose").getDoubleArray(defaultPose); // Robot pose relative to field based on detected tag
  
          // Display Limelight values on the SmartDashboard
          SmartDashboard.putNumber("LimelightX", tx);
          SmartDashboard.putNumber("LimelightY", ty);
          SmartDashboard.putNumber("LimelightArea", tid);
          SmartDashboard.putNumber ("April Tag Detected", tid);
          // Check if the detected tag is specifically ID 4
          if (tid == 4) {
              // Display AprilTag ID and pose information on the SmartDashboard for ID 4
              SmartDashboard.putNumber("Detected AprilTag ID", tid);
              double x = botPose[0]; // X position relative to the tag
              double y = botPose[1]; // Y position relative to the tag
              double z = botPose[2]; // Z position (distance) relative to the tag
              double roll = botPose[3]; // Rotation around X-axis
              double pitch = botPose[4]; // Rotation around Y-axis
              double yaw = botPose[5]; // Rotation around Z-axis
  
              // Display detailed pose information for AprilTag ID 4
              SmartDashboard.putNumber("Tag 4 X", x);
              SmartDashboard.putNumber("Tag 4 Y", y);
              SmartDashboard.putNumber("Tag 4 Z", z);
              SmartDashboard.putNumber("Tag 4 Roll", roll);
              SmartDashboard.putNumber("Tag 4 Pitch", pitch);
              SmartDashboard.putNumber("Tag 4 Yaw", yaw);
              SmartDashboard.putString("Tag 4 Status", "Detected");
  
              // Implement control logic when AprilTag ID 4 is detected, such as alignment or positioning.
          }
      }
  
  }
	/**
	 * if the robot is not in hatMode and in normal drive, the LED turns solid white (0.93)
	 */

	

  // Limelight network table
  private NetworkTable limelightTable;

  Joystick stick = new Joystick(2);
  Joystick driverController = new Joystick(1);
  Spark blinkin = new Spark(0);
  
  CANSparkMax shooterPivot = new CANSparkMax(9, MotorType.kBrushless);
  CANSparkMax intakePivot = new CANSparkMax(18, MotorType.kBrushless);
  CANSparkMax intakeAxles = new CANSparkMax(11, MotorType.kBrushless);
  CANSparkMax secondIntakeAxles = new CANSparkMax(19, MotorType.kBrushless);
  CANSparkMax leftShooterBelt = new CANSparkMax(12, MotorType.kBrushless);
  CANSparkMax rightShooterBelt = new CANSparkMax(13, MotorType.kBrushless);
  CANSparkMax rightShooterWheel = new CANSparkMax(14, MotorType.kBrushless);
  CANSparkMax leftShooterWheel = new CANSparkMax(15, MotorType.kBrushless);
  CANSparkMax liftyLeft = new CANSparkMax(17, MotorType.kBrushless);
  CANSparkMax liftyRight = new CANSparkMax(16, MotorType.kBrushless);


  private static final String kDefaultAuto = "2 Note Auto Center";
  private static final String kCustomAuto = "Shoot one (Blue source, Red Amp)";
  private static final String kCustomAuto2 = "3 Note Auto";
  private static final String kCustomAuto3 = "Leave & Return Starting Zone";
  private static final String kCustomAuto4 = "4 Note Auto";
  private static final String kCustomAuto5 = "Shoot one (Blue Amp, Red Source)";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser <>();

  double shooter_joystick_speed = 0.01;
  double intake_joystick_speed =  0.01;
  enum SystemState { UserControl, Handoff1, Handoff2, Handoff3 };


 SystemState system_state = SystemState.UserControl;

 SlewRateLimiter shooter_rate_limiter = new SlewRateLimiter(1); // 90 deg per second
 SlewRateLimiter intake_rate_limiter = new SlewRateLimiter(1); // 90 deg per second

 PIDController shooter_pos_pid = new PIDController(2.5, 0.0, 0.0);
 PIDController intake_pos_pid = new PIDController(2.5, 0.0, 0.0);

  double intake_setpoint_lower_limit = 0.479;
  double intake_setpoint_upper_limit = 0.948;

  double shooter_setpoint_lower_limit = 0.487;
  double shooter_setpoint_upper_limit = 0.99;

  public static double funVariable = 4;

  public double intake_setpoint = 0;
  public double shooter_setpoint = 0;


  public double tx;
  public double ty;
  public double ta;
  public double tv;

  DigitalInput limitSwitch = new DigitalInput(0);

  public Timer autonomy_timer = new Timer();
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;


  @Override
  public void robotInit() {

    SmartDashboard.putData("Auto Choices", m_chooser);
    m_chooser.setDefaultOption("2 Note Auto Center", kDefaultAuto);
    m_chooser.addOption("Shoot one (Blue source, Red Amp)", kCustomAuto);
    m_chooser.addOption("3 Note Auto", kCustomAuto2);
    m_chooser.addOption("Leave & Return Starting Zone", kCustomAuto3);
    m_chooser.addOption("4 Note Auto", kCustomAuto4);
    m_chooser.addOption("Shoot one (Blue Amp, Red Source)", kCustomAuto5);
    CameraServer.startAutomaticCapture();

    // Initialize the Limelight network table
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

   /*
    UsbCamera usbCamera0 = new UsbCamera("Camera 0", 0);
    UsbCamera usbCamera1 = new UsbCamera("Camera 1", 1);

    MjpegServer mjpegServer0 = new MjpegServer("Camera 0", 0);
    MjpegServer mjpegServer1 = new MjpegServer("Camera 1", 1);

    mjpegServer0.setSource(usbCamera0);
    mjpegServer1.setSource(usbCamera1);

    CvSink cvSink = new CvSink("Camera 0");
    cvSink.setSource(usbCamera0);

    CvSink cvSink1 = new CvSink("Camera 1");
    cvSink1.setSource(usbCamera1);

    CvSource outputStream0 = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
    CvSource outputStream1 = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);

    MjpegServer mjpegServer2 = new MjpegServer("severeblur", 1182);
    MjpegServer mjpegServer3 = new MjpegServer("severeblur", 1183);

    mjpegServer2.setSource(outputStream0);
    mjpegServer3.setSource(outputStream1);
    */


    m_autoSelected = m_chooser.getSelected();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    homeSetpoints();
  }

  public void resetLimitSwitch() {
    limitSwitch.get();
  }

  public void homeSetpoints() {
    shooter_setpoint = getShooterFeedback();
    intake_setpoint = getIntakeFeedback();
  }

  public double wrapAngle(double ang) {
    return Math.atan2(Math.sin(ang), Math.cos(ang));
  }

  public double getShooterAngle() {
      double pos = shooterPivot.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
      if (pos < 0.01) {
        pos = 1;
      }
      return pos;
  }
  


  public double getIntakeAngle() {
    return intakePivot.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
  }

  public double getShooterFeedback() {
    // Offset so horizontal angle is 90 deg
    return getShooterAngle();
  }

  public double getIntakeFeedback() {
    // Offset so the centered angle is pi
    return getIntakeAngle();
  }

  public void clampSetpoints() {
    if (shooter_setpoint < shooter_setpoint_lower_limit) { shooter_setpoint = shooter_setpoint_lower_limit; }
    if (shooter_setpoint > shooter_setpoint_upper_limit) {shooter_setpoint = shooter_setpoint_upper_limit; }

    if (intake_setpoint < intake_setpoint_lower_limit) { intake_setpoint = intake_setpoint_lower_limit; }
    if (intake_setpoint > intake_setpoint_upper_limit) { intake_setpoint = intake_setpoint_upper_limit; }
  }

  public void controlIntake() {
    double rate_limited_setpoint = intake_rate_limiter.calculate(intake_setpoint);
    double intake_cmd = intake_pos_pid.calculate(getIntakeFeedback(), rate_limited_setpoint);
    intakePivot.set(intake_cmd);
  }

  public void controlShooter() {
    double rate_limited_setpoint = shooter_rate_limiter.calculate(shooter_setpoint);
    double shooter_cmd = shooter_pos_pid.calculate(getShooterFeedback(), rate_limited_setpoint);
    shooterPivot.set(-shooter_cmd);
  }

  public void arbitrateSetpoints() {
    double stick_x = stick.getRawAxis(1);
    double stick_y = stick.getRawAxis(5);
    
    if (Math.abs(stick_x) > 0.1) {
      shooter_setpoint = shooter_setpoint + stick_x * shooter_joystick_speed;
    }
  
    if (Math.abs(stick_y) > 0.1) {
      intake_setpoint = intake_setpoint + stick_y * intake_joystick_speed;
    }

    if (stick.getRawButton(4)) {
      //amp
      shooter_setpoint = 0.6458;
    }
    else if (stick.getRawButton(3)) {
      //speaker scoring/ handoff
      intake_setpoint = 0.481;
      shooter_setpoint = 0.86;//.87
    }
    else if (stick.getRawButton(1)) {
      //intaking
      intake_setpoint = 0.923; //933
    }
    else if (stick.getRawButton(8)) {
      //speaker scoring/ handoff
      shooter_setpoint = 0.88;
    }
  }
  
  // The final handoff state should return the system state to user control
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
    SmartDashboard.putNumber("shooter encoder", getShooterAngle());
    SmartDashboard.putNumber("intake encoder", getIntakeAngle());
    SmartDashboard.putNumber("intake setpoint", intake_setpoint);
    SmartDashboard.putNumber("shooter setpoint", shooter_setpoint);
    SmartDashboard.putNumber("LimelightX", tx);
    SmartDashboard.putNumber("LimelightY", ty);
    SmartDashboard.putNumber("LimelightArea", ta);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
   
  }

  @Override
  public void disabledPeriodic() {

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto Selected:" + m_autoSelected);
      autonomy_timer.reset();
      autonomy_timer.start();

      switch(m_autoSelected) {
        //2notecenter
        case kDefaultAuto:
        m_autonomousCommand = m_robotContainer.testAutoCommand();
        break;

        //right2note
        case kCustomAuto:
        m_autonomousCommand = m_robotContainer.testAutoCommand2();
        break;

        //3noteauto
        case kCustomAuto2:
        m_autonomousCommand = m_robotContainer.testAutoCommand3();
        break;

        //taxi
        case kCustomAuto3:
        m_autonomousCommand = m_robotContainer.testAutoCommand4();
        break;

        //4noteauto
        case kCustomAuto4:
        m_autonomousCommand = m_robotContainer.testAutoCommand5();
        break;
        case kCustomAuto5:
        m_autonomousCommand = m_robotContainer.testAutoCommand6();
        break;
      }
        
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    switch (m_autoSelected) {
      //2notecenter
      case kDefaultAuto:
       if (autonomy_timer.hasElapsed(15)) {
          intakeAxles.set(0);
          secondIntakeAxles.set(0);
          leftShooterBelt.set(0);
          rightShooterBelt.set(0);
          leftShooterWheel.set(0);
          rightShooterWheel.set(0);
        }
         else if (autonomy_timer.hasElapsed(13)) {
         intake_setpoint = 0.924;
        }

       else if (autonomy_timer.hasElapsed(6.5)) {
          leftShooterBelt.set(-1);
          rightShooterBelt.set(1);
          shooter_setpoint = 0.88;
        }
        
        else if(autonomy_timer.hasElapsed(6.1)) {
          intakeAxles.set(-1);
          secondIntakeAxles.set(1);
          shooter_setpoint = 0.88;
        }

        else if (autonomy_timer.hasElapsed(5)) {
          leftShooterWheel.set(-.60);
          rightShooterWheel.set(.60);
          intake_setpoint = 0.481;
          //4.25
        }
        else if (autonomy_timer.hasElapsed(4.1)) {//3.25
          intakeAxles.set(0);
          secondIntakeAxles.set(0);
        }
        else if (autonomy_timer.hasElapsed(3)) {
          leftShooterWheel.set(0);
          rightShooterWheel.set(0);
          leftShooterBelt.set(0);
          rightShooterBelt.set(0);
        }
        else if (autonomy_timer.hasElapsed(1.5)) {
          leftShooterBelt.set(-.60);
          rightShooterBelt.set(.60);
          intakeAxles.set(1);
          secondIntakeAxles.set(-1);
        }
        else if (autonomy_timer.hasElapsed(.01)) {
          shooter_setpoint = 0.878;
          leftShooterWheel.set(-.60);
          rightShooterWheel.set(.60);
          intake_setpoint = 0.924; //.93 

        }

        clampSetpoints();
        controlIntake();
        controlShooter();
        break;
/* BREAK */
        //right2note
        case kCustomAuto:
        if (autonomy_timer.hasElapsed(10)) {
          intakeAxles.set(0);
          secondIntakeAxles.set(0);
          leftShooterBelt.set(0);
          rightShooterBelt.set(0);
          leftShooterWheel.set(0);
          rightShooterWheel.set(0);
        }
        else if (autonomy_timer.hasElapsed(2.5)) {
          leftShooterBelt.set(-1);
          rightShooterBelt.set(1);
        }
        else if (autonomy_timer.hasElapsed(.01)) {
          shooter_setpoint = .86; //.8956
          leftShooterWheel.set(-.90);
          rightShooterWheel.set(.90);
          intake_setpoint = 0.7; //.93 

        }
        clampSetpoints();
        controlIntake();
        controlShooter();
        break;
/* BREAK */
        //3noteauto
        case kCustomAuto2: 
        if (autonomy_timer.hasElapsed(15)) {
          intakeAxles.set(0);
          secondIntakeAxles.set(0);
          leftShooterBelt.set(0);
          rightShooterBelt.set(0);
          leftShooterWheel.set(0);
          rightShooterWheel.set(0);
        }

        else if (autonomy_timer.hasElapsed(13)) {
          intakeAxles.set(0);
          secondIntakeAxles.set(0);
          intake_setpoint = 0.933;
        }

        else if (autonomy_timer.hasElapsed(11)) {
          leftShooterBelt.set(-1);
          rightShooterBelt.set(1);
          shooter_setpoint = 0.874;
        }

        else if (autonomy_timer.hasElapsed(10)) {
          leftShooterWheel.set(-.90);
          rightShooterWheel.set(.90);
           intakeAxles.set(-1);
           secondIntakeAxles.set(1);
          
        }

          
        else if (autonomy_timer.hasElapsed(8.6)) {
          shooter_setpoint = .878;
          intake_setpoint = 0.481;
          intakeAxles.set(0);
           //.93 
        }

       else if (autonomy_timer.hasElapsed(8)) {
          leftShooterBelt.set(0);
          rightShooterBelt.set(0);
          leftShooterWheel.set(0);
          rightShooterWheel.set(0);
        }

      else if (autonomy_timer.hasElapsed(7)) {
          intakeAxles.set(1);
          secondIntakeAxles.set(-1);
        }

        else if (autonomy_timer.hasElapsed(6)) {
          intake_setpoint = 0.933;
        }
        else if (autonomy_timer.hasElapsed(4.25)) {
         intakeAxles.set(-1);
         secondIntakeAxles.set(1);
        }
       else if (autonomy_timer.hasElapsed(4)) {
          shooter_setpoint = 0.88;
        }

        else if (autonomy_timer.hasElapsed(3.75)) {
          leftShooterBelt.set(-1);
          rightShooterBelt.set(1);
        }

        else if (autonomy_timer.hasElapsed(3.5)) {
          leftShooterWheel.set(-.80);
          rightShooterWheel.set(.80);
          intakeAxles.set(0);  
          secondIntakeAxles.set(0);
        }

        else if (autonomy_timer.hasElapsed(3)) {
         shooter_setpoint = 0.880;
         intake_setpoint = 0.481;
 
        }

        else if (autonomy_timer.hasElapsed(2.5)) {
          leftShooterWheel.set(0);
          rightShooterWheel.set(0);
          leftShooterBelt.set(0);
          rightShooterBelt.set(0);
        }

        else if (autonomy_timer.hasElapsed(1)) {
          leftShooterBelt.set(-.80);
          rightShooterBelt.set(.80);
          intakeAxles.set(1);
          secondIntakeAxles.set(-1);
        }

        else if (autonomy_timer.hasElapsed(.01)) {
          shooter_setpoint = 0.85;
          leftShooterWheel.set(-.95);
          rightShooterWheel.set(.95);
          intake_setpoint = 0.933; //.924
        }

        clampSetpoints();
        controlIntake();
        controlShooter();
        break;
        //taxi
        case kCustomAuto3:
        if (autonomy_timer.hasElapsed(2)){
        intakeAxles.set(0);
        }

        clampSetpoints();
        controlIntake();
        controlShooter();
        break;
      
      //4note
      case kCustomAuto4:
          blinkin.set(-0.67);
        if (autonomy_timer.hasElapsed(15)) {
          intakeAxles.set(0);
          secondIntakeAxles.set(0);
          leftShooterBelt.set(0);
          rightShooterBelt.set(0);
          leftShooterWheel.set(0);
          rightShooterWheel.set(0);
        }
        else if (autonomy_timer.hasElapsed(13.75)) {
          intakeAxles.set(-1);
        }

        else if (autonomy_timer.hasElapsed(13)) {
          leftShooterBelt.set(-1);
          rightShooterBelt.set(1);
        }

        else if (autonomy_timer.hasElapsed(12.5)) {
         shooter_setpoint = 0.88;
         intake_setpoint = 0.481;
         intakeAxles.set(0);
         leftShooterWheel.set(-.85);
         rightShooterWheel.set(.85);
        }

        else if (autonomy_timer.hasElapsed(9.5)){
          intakeAxles.set(1);
          intake_setpoint = 0.933;  
        }

        else if (autonomy_timer.hasElapsed(8.8)) {
          leftShooterBelt.set(-1);
          rightShooterBelt.set(1);
          intakeAxles.set(-1);
        }

        else if (autonomy_timer.hasElapsed(8)) {
          shooter_setpoint = 0.88;
          leftShooterWheel.set(-.85);
          rightShooterWheel.set(.85);
        }

        else if (autonomy_timer.hasElapsed(7.9)) {
         shooter_setpoint = 0.88;
         intake_setpoint = 0.481;
        }

        else if (autonomy_timer.hasElapsed(7)) {
         intakeAxles.set(0);
        }


        // second shot done

        else if (autonomy_timer.hasElapsed(6)) {
          intakeAxles.set(1); 
        }

        else if (autonomy_timer.hasElapsed(5.3)) {
          leftShooterWheel.set(0);
          rightShooterWheel.set(0);
          leftShooterBelt.set(0);
          rightShooterBelt.set(0); 
          intake_setpoint = 0.933;
 
        }

        else if (autonomy_timer.hasElapsed(4)) {
          leftShooterBelt.set(-1);
          rightShooterBelt.set(1);
          intakeAxles.set(-1);  
        }

        else if (autonomy_timer.hasElapsed(3.2)) {
          leftShooterWheel.set(-.85);
          rightShooterWheel.set(.85);
        }

        else if (autonomy_timer.hasElapsed(3)) {
         shooter_setpoint = 0.88;
         intake_setpoint = 0.481;
         intakeAxles.set(0);
        }

// 1st shot done
        else if (autonomy_timer.hasElapsed(2)) {
          leftShooterWheel.set(0);
          rightShooterWheel.set(0);
          leftShooterBelt.set(0);
          rightShooterBelt.set(0);
        }

        else if (autonomy_timer.hasElapsed(1)) {
          leftShooterBelt.set(-1);
          rightShooterBelt.set(1);
          intakeAxles.set(1);
        }

        else if (autonomy_timer.hasElapsed(.01)) {
          shooter_setpoint = 0.88;
          leftShooterWheel.set(-.85);
          rightShooterWheel.set(.85);
          intake_setpoint = 0.933; //.924
        }

        clampSetpoints();
        controlIntake();
        controlShooter();
        break;
    
      case kCustomAuto5:
        if (autonomy_timer.hasElapsed(10)) {
          intakeAxles.set(0);
          secondIntakeAxles.set(0);
          leftShooterBelt.set(0);
          rightShooterBelt.set(0);
          leftShooterWheel.set(0);
          rightShooterWheel.set(0);
        }
        else if (autonomy_timer.hasElapsed(2.5)) {
          leftShooterBelt.set(-1);
          rightShooterBelt.set(1);
        }
        else if (autonomy_timer.hasElapsed(.01)) {
          shooter_setpoint = .86; //.8956
          leftShooterWheel.set(-.90);
          rightShooterWheel.set(.90);
          intake_setpoint = 0.7; //.93 

        }
        clampSetpoints();
        controlIntake();
        controlShooter();
        break;
    }
  }
  
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    homeSetpoints();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // Display values on the SmartDashboard for debugging
    SmartDashboard.putNumber("LimelightX", tx);
    SmartDashboard.putNumber("LimelightY", ty);
    SmartDashboard.putNumber("LimelightArea", ta);
    SmartDashboard.putBoolean("TargetVisible", tv == 1.0);

    tx = limelightTable.getEntry("tx").getDouble(0.0); // Horizontal offset from crosshair to target
    ty = limelightTable.getEntry("ty").getDouble(0.0); // Vertical offset from crosshair to target
    ta = limelightTable.getEntry("ta").getDouble(0.0); // Target area
    tv = limelightTable.getEntry("tv").getDouble(0.0); // Target validity



    // Check if Limelight sees a valid target
    /* Do NOT run unless object detected
     *
     * This is counterintuitive (LOL) but axles would run 
     * constantly otherwise and thats lowkey inconvenient
     * 
     */

    /*  while (limitSwitch.get() == false) {
          intakeAxles.set(1);
        }
    */

    blinkin.set(0.57);

    if (driverController.getRawButton(1)) {
       blinkin.set(-0.67);
    }
    else if (driverController.getRawButton(2)) {
      blinkin.set(0.57);
    }

    arbitrateSetpoints();
    clampSetpoints();
    controlIntake();
    controlShooter();

    //SHOOTER WHEELS
    double right_trigger = stick.getRawAxis(3);
    double left_trigger = stick.getRawAxis(2);

    //JOYSTICK CONTROLL
     if (Math.abs(right_trigger) > 0.1) {
      //wheels out
        rightShooterWheel.set(0.95);
        leftShooterWheel.set(-0.95);
      }
     else if (Math.abs(left_trigger) > 0.1) {
      //wheels in
        rightShooterWheel.set(-0.10);
        leftShooterWheel.set(0.10);
      }
      else {
      rightShooterWheel.set(0);
      leftShooterWheel.set(0);
      }
    

    //INTAKE AXLE
    if (stick.getRawButton(6)) {      
    //NOTE out

      intakeAxles.set(-1);
      secondIntakeAxles.set(1);
    }
    else if (stick.getRawButton(5)) {
    //NOTE in
      intakeAxles.set(1);
      secondIntakeAxles.set(-1);

      if (limitSwitch.get() == false) {
        intakeAxles.set(0);
      }
    }

    else {
      intakeAxles.set(0);
      secondIntakeAxles.set(0);
    }

    // Limit Switch Hard Stop. Intake UNTIL limit switch detects object
    // HAVEN'T TESTED THIS YET NO IDEA IF IT WORKS :

    /*
    else if (stick.getRawButton(5) && limitSwitch.get() == false) {
      intakeAxles.set(0);
      secondIntakeAxles.set(0);
    } 
    */
    

    
    //SHOOTER BELT IN
    if (stick.getRawButton(2)){
      rightShooterBelt.set(0.85);
      leftShooterBelt.set(-0.85);}
    //SHOOTER BELT OUT
    else if (stick.getRawButton(7)){
      rightShooterBelt.set(-0.05);
      leftShooterBelt.set(0.05);
    }
    else {
      rightShooterBelt.set(0);
      leftShooterBelt.set(0);
    }
      
    double left_Trigger = driverController.getRawAxis(2);
    double right_Trigger = driverController.getRawAxis(3);

    if (Math.abs(left_Trigger) > 0.1) {
      //Climbers up
      liftyLeft.set(-0.5);//4
      liftyRight.set(0.5);//4
    }
    else if (Math.abs(right_Trigger) > 0.1)  {

      //Climbers down
      liftyLeft.set(0.85);//75
      liftyRight.set(-0.85);//75
    }
    else if (driverController.getRawButton(1)) {
     // blinkin.lightsAmplify();
    }

    else {
      //STOP
      liftyLeft.set(0);
      liftyRight.set(0);
    }
  }
  
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}

