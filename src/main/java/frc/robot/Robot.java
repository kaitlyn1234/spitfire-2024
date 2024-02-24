// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.CANSparkMax;
//import edu.wpi.first.wpilibj.Spark;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
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

  

 SlewRateLimiter lift_rate_limiter = new SlewRateLimiter(Math.PI / 2.0); // 90 deg per second
 SlewRateLimiter wrist_rate_limiter = new SlewRateLimiter(Math.PI / 2.0); // 90 deg per second

 PIDController shooter_pos_pid = new PIDController(0.25, 0.0, 0.0);
 PIDController intake_pos_pid = new PIDController(0.1, 0.0, 0.0);
/* 
  double lift_setpoint_lower_limit = 0.3;
  double lift_setpoint_upper_limit = 5.6;
  double wrist_setpoint_lower_limit = 0.45;
  double wrist_setpoint_upper_limit = 5.45;

  double wrist_joystick_speed = 0.015;
  double lift_joystick_speed = 0.01;

  public double wrist_setpoint = 0;
  public double lift_setpoint = 0;
*/

/*
  double intake_setpoint_lower_limit = 0;
  double intake_setpoint_upper_limit = 0;

  double shooter_setpoint_lower_limit = 0;
  double shooter_setpoint_upper_limit = 0;

  public double intake_setpoint = 0;
  public double shooter_setpoint = 0;
  */

  public Timer autonomy_timer = new Timer();

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;


  @Override
  public void robotInit() {

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);


    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

   // homeSetpoints();
  }
/* 
  public void homeSetpoints() {
    lift_setpoint = getLiftFeedback();
    wrist_setpoint = getWristFeedback();
  }
*/
  public double wrapAngle(double ang) {
    return Math.atan2(Math.sin(ang), Math.cos(ang));
  }

  public double getShooterAngle() {
      return wrapAngle(-shooterPivot.getAbsoluteEncoder(Type.kDutyCycle).getPosition() * Math.PI * 2);
  }

  public double getIntakeAngle() {
    return wrapAngle(-intakePivot.getAbsoluteEncoder(Type.kDutyCycle).getPosition() * Math.PI * 2 - Math.PI / 2.0);
  }
/* 
  public double getLiftFeedback() {
    // Offset so horizontal angle is 90 deg
    return getLiftAngle() + Math.PI;
  }

  public double getWristFeedback() {
    // Offset so the centered angle is pi
    return getWristAngle() + Math.PI;
  }

  public void clampSetpoints() {
    if (lift_setpoint < lift_setpoint_lower_limit) { lift_setpoint = lift_setpoint_lower_limit; }
    if (lift_setpoint > lift_setpoint_upper_limit) { lift_setpoint = lift_setpoint_upper_limit; }

    if (wrist_setpoint < wrist_setpoint_lower_limit) { wrist_setpoint = wrist_setpoint_lower_limit; }
    if (wrist_setpoint > wrist_setpoint_upper_limit) { wrist_setpoint = wrist_setpoint_upper_limit; }
  }

  public void controlWrist() {
    double rate_limited_setpoint = wrist_rate_limiter.calculate(wrist_setpoint);
    double wrist_cmd = wrist_pos_pid.calculate(getWristFeedback(), rate_limited_setpoint);
    wrist.set(-wrist_cmd);
  }

  public double getLiftFF() {
    return 0.04 * Math.cos(getLiftFeedback());
  }

  public void controlLift() {
    double rate_limited_setpoint = lift_rate_limiter.calculate(lift_setpoint);
    double lift_fb = getLiftFeedback();
    double lift_cmd = lift_pos_pid.calculate(lift_fb, rate_limited_setpoint);
    lift_cmd = lift_cmd + getLiftFF();
    rightliftmotor.set(-lift_cmd);
    leftliftmotor.set(lift_cmd);
  }

  public void arbitrateSetpoints() {
    if (stick.getRawButton(9)) {
      wrist_setpoint = Math.PI; // Straight
      lift_setpoint = Math.PI / 2; // Parallel to floor
    }
     
    else if (stick.getRawButton(1)){
      //cone scoring
      wrist_setpoint = 1.75;
      lift_setpoint = 2.28;
    }

    else if (stick.getRawButton(8)){
      //home
      wrist_setpoint = 0.525;
      lift_setpoint = 5.568;
    }

    else if (stick.getRawButton(2)){
      //cube scoring
      wrist_setpoint = 3.564;
      lift_setpoint = 1.828;
    }

    else if (stick.getRawButton(3)){
      //cone human player
      wrist_setpoint = 1.205;
      lift_setpoint = 5.23;
    }

    else if (stick.getRawButton(4)){
      //cube human player
      wrist_setpoint = 3.019;
      lift_setpoint = 4.961;
    }
   
    else {
      double stick_x = stick.getRawAxis(1);
      double stick_y = stick.getRawAxis(5);
      if (Math.abs(stick_x) > 0.1) {
        lift_setpoint = lift_setpoint + stick_x * lift_joystick_speed;
      }
  
      if (Math.abs(stick_y) > 0.1) {
        wrist_setpoint = wrist_setpoint + stick_y * wrist_joystick_speed;
      }
    }
  }*/

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
    SmartDashboard.putNumber("shooter encoder", getShooterAngle());
    SmartDashboard.putNumber("intake encoder", getIntakeAngle());
/* 
    SmartDashboard.putNumber("lift feedback", getLiftFeedback());
    SmartDashboard.putNumber("wrist feedback", getWristFeedback());*/
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
   // homeSetpoints();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    /*arbitrateSetpoints();
    clampSetpoints();
    controlWrist();
    controlLift();*/


    /*//SHOOTER BELTS
    if (stick.getRawButton(1)) {
      //belt OUT
      rightShooterBelt.set(0.11);
      leftShooterBelt.set(-0.11);
    }
    else if (stick.getRawButton(2)) {
      //blet IN 
      rightShooterBelt.set(-0.11);
      leftShooterBelt.set(0.11);
    }
    else {
      rightShooterBelt.set(0);
      leftShooterBelt.set(0);
    }*/


    //SHOOTER WHEELS
    if (stick.getRawButton(3)) {
      //shooter IN
      rightShooterWheel.set(0.11);
      leftShooterWheel.set(-0.11);
    }
      else if (stick.getRawButton(4)) {
      //SHOOTER out
      rightShooterWheel.set(-0.11);
      leftShooterWheel.set(0.11);
    }
    else {
      rightShooterWheel.set(0);
      leftShooterWheel.set(0);
    }


     //SHOOTER PIVOT
    if (stick.getRawButton(5)) {
      //PIVOT OUT
      shooterPivot.set(0.11);
    }
    else if (stick.getRawButton(6)) {
      //PIVOT IN 
      shooterPivot.set(-0.11);
    }
    else {
      //STOP
      shooterPivot.set(0);
    }
    
     //INTAKE PIVOT
    if (stick.getRawButton(7)) {
      //PIVOT OUT
      intakePivot.set(0.20);
    }
    else if (stick.getRawButton(8)) {
      //PIVOT IN 
      intakePivot.set(-0.20);
    }
    else {
      //STOP
      intakePivot.set(0);
    }

        //INTAKE AXLE
    if (stick.getRawButton(9)) {
      //NOTE OUT
      intakeAxles.set(1);
    }
    else if (stick.getRawButton(10)) {
      //NOTE IN 
      intakeAxles.set(-1);
    }
    else {
      //STOP
      intakeAxles.set(0);

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
    if (stick.getRawButton(1)) {
      //CLIMBERS DOWN
      liftyLeft.set(-0.2);
      liftyRight.set(0.2);
    }
    else if (stick.getRawButton(2)) {
      //CLIMBERS UP
      liftyLeft.set(0.2);
      liftyRight.set(-0.2);
    }
    else {
      liftyLeft.set(0);
      liftyRight.set(0);}

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
