// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.CANSparkMax;
//import edu.wpi.first.wpilibj.Spark;
import com.revrobotics.EncoderType;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  Joystick stick = new Joystick(2);
  
  CANSparkMax shooterPivot = new CANSparkMax(9, MotorType.kBrushless);
  CANSparkMax intakePivot = new CANSparkMax(18, MotorType.kBrushless);
  CANSparkMax intakeAxles = new CANSparkMax(11, MotorType.kBrushless);
  CANSparkMax leftShooterBelt = new CANSparkMax(12, MotorType.kBrushless);
  CANSparkMax rightShooterBelt = new CANSparkMax(13, MotorType.kBrushless);
  CANSparkMax rightShooterWheel = new CANSparkMax(14, MotorType.kBrushless);
  CANSparkMax leftShooterWheel = new CANSparkMax(15, MotorType.kBrushless);
  CANSparkMax liftyLeft = new CANSparkMax(17, MotorType.kBrushless);
  CANSparkMax liftyRight = new CANSparkMax(16, MotorType.kBrushless);

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser <>();

  double shooter_joystick_speed = 0.01;
  double intake_joystick_speed =  0.01;
  enum SystemState { UserControl, Handoff1, Handoff2, Handoff3 };

  SystemState system_state = SystemState.UserControl;
/* 
public class Blinkin {

  Spark blinkin = new Spark(0);
  Joystick driverController = new Joystick(0);
  public Blinkin() {
  }
  
  public void lightsCone() {
    blinkin.set(0.69);
  }
  public void lightsCube() {
    blinkin.set(0.91);
  }
  public void normalight() {
    blinkin.set(0.57);
  }

  public void teleopPeriodic() {
  if(driverController.getRawButton(6)){
  lightsCone();
  }
  else if(driverController.getRawButton(4)){
  lightsCube();
  }
  else{
  normalight();
  }
  }}*/

  

 SlewRateLimiter shooter_rate_limiter = new SlewRateLimiter(1); // 90 deg per second
 SlewRateLimiter intake_rate_limiter = new SlewRateLimiter(1); // 90 deg per second

 PIDController shooter_pos_pid = new PIDController(2.5, 0.0, 0.0);
 PIDController intake_pos_pid = new PIDController(2.5, 0.0, 0.0);



  double intake_setpoint_lower_limit = 0.479;
  double intake_setpoint_upper_limit = 0.948;

  double shooter_setpoint_lower_limit = 0.487;
  double shooter_setpoint_upper_limit = 0.99;

  public double intake_setpoint = 0;
  public double shooter_setpoint = 0;

  

  public Timer autonomy_timer = new Timer();

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;


  @Override
  public void robotInit() {

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);


    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

   homeSetpoints();
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

  public boolean setpointsAchieved() {
    double setpoint_tol = 0.05;
    boolean intake_achieved = Math.abs(intake_setpoint - getIntakeAngle()) < setpoint_tol;

    boolean shooter_achieved = false; // do the same thing for the shooter! (add the abs check)

    return intake_achieved && shooter_achieved;
  }

  public void arbitrateSetpoints() {
  /*  if (system_state == SystemState.UserControl) {
      double stick_x = stick.getRawAxis(1);
      double stick_y = stick.getRawAxis(5);

      if (Math.abs(stick_x) > 0.1) {
        shooter_setpoint = shooter_setpoint + stick_x * shooter_joystick_speed;
      }
  
      if (Math.abs(stick_y) > 0.1) {
        intake_setpoint = intake_setpoint + stick_y * intake_joystick_speed;
      }
      if (// handoff button pressed  false) {
       // system_state = SystemState.Handoff1;
      }
      if (/* shoot position button pressed */ //false) {
        //system_state = SystemState.UserControl; // switch to shoot position state
     // }
   //}
  /*  else if (system_state == SystemState.Handoff1) {
      shooter_setpoint = 0; // something not zero
      intake_setpoint = 0; // something not zero
      if (setpointsAchieved()) {
         // system_state = SystemState.Handoff2
      }*/
   //}
      double stick_x = stick.getRawAxis(1);
      double stick_y = stick.getRawAxis(5);
    



      if (Math.abs(stick_x) > 0.1) {
        shooter_setpoint = shooter_setpoint + stick_x * shooter_joystick_speed;
      }
  
      if (Math.abs(stick_y) > 0.1) {
        intake_setpoint = intake_setpoint + stick_y * intake_joystick_speed;
      }


    if (stick.getRawButton(3)) {
      //amp
      shooter_setpoint = 0.6624;
    }
     
    else if (stick.getRawButton(4)){
      //speaker scoring/ handoff
      intake_setpoint = 0.481;
      shooter_setpoint = 0.898;
    
    }

    else if (stick.getRawButton(1)){
      //intaking
      intake_setpoint = 0.924;
    
    
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

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    /*wrist_setpoint = 1.75;
    lift_setpoint = 2.28;

    if (autonomy_timer.hasElapsed(10)) {
      rightintake.set(0.0);
      leftintake.set(0.0);
    }
    else if (autonomy_timer.hasElapsed(5)) {
      rightintake.set(-0.3);
      leftintake.set(-0.3);
    }
    
    clampSetpoints();
    controlWrist();
    controlLift();
    */
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

    arbitrateSetpoints();
    clampSetpoints();
    controlIntake();
    controlShooter();


    //SHOOTER WHEELS
    double right_trigger = stick.getRawAxis(3);
    double left_trigger = stick.getRawAxis(2);


     if (Math.abs(right_trigger) > 0.1) {
      //wheels out
        rightShooterWheel.set(0.60);
        leftShooterWheel.set(-0.60);
        intake_setpoint = 0.614;
      }
  
     else if (Math.abs(left_trigger) > 0.1) {
      //wheels in
        rightShooterWheel.set(-0.30);
        leftShooterWheel.set(0.30);
      }

      else {
      rightShooterWheel.set(0);
      leftShooterWheel.set(0);
    }



        //INTAKE AXLE
    if (stick.getRawButton(5)) {
      //NOTE OUT
      intakeAxles.set(1);
    }
    else if (stick.getRawButton(6)) {
      //NOTE IN
      //belt in 
      intakeAxles.set(-1);
      rightShooterBelt.set(0.40);
      leftShooterBelt.set(-0.40);
    }
    else if (stick.getRawButton(7)) {
      //blet out
      rightShooterBelt.set(-0.20);
      leftShooterBelt.set(0.20);
    }
    
    else {
      //STOP
      intakeAxles.set(0);
      rightShooterBelt.set(0);
      leftShooterBelt.set(0);

    }
    




     /*  double stick_x = stick.getRawAxis(4);
      double stick_y = stick.getRawAxis(3);

      if (Math.abs(stick_x) > 0.1) {
      //Climbers up
      liftyLeft.set(0.2);
      liftyRight.set(0.2);
    }
    else if (Math.abs(stick_y) > 0.1)  {

      //Climbers down
      liftyLeft.set(0.2);
      liftyRight.set(-0.2);
    }
    else {
      //STOP
      liftyLeft.set(0);
      liftyRight.set(0);
    }*/
   
     /*  int povUp = stick.getPOV(0);
      int povDown = stick.getPOV(180);
      int POVnotPressed = stick.getPOV(-1);

    if (stick.getRawButton(povUp)){
      liftyLeft.set(0.50);
      liftyRight.set(-0.50);
    }

   else if (stick.getRawButton(povDown)){
      liftyLeft.set(-0.50);
      liftyRight.set(0.50);}

   else if (stick.getRawButton(P)){
      liftyLeft.set(0);
      liftyRight.set(0);}
   
   
   
   if (stick.getRawButton(1)) {
      //CLIMBERS DOWN
      liftyLeft.set(-0.50);
      liftyRight.set(0.50);
    }
    else if (stick.getRawButton(2)) {
      //CLIMBERS UP
      liftyLeft.set(0.50);
      liftyRight.set(-0.50);
    }
    else {
      liftyLeft.set(0);
      liftyRight.set(0);}
*/
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
