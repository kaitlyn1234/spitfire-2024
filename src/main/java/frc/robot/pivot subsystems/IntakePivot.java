// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {

  private final CANSparkMax m_intakePivot;
  private final AbsoluteEncoder m_intakeEncoder;
  private final SparkMaxPIDController m_intakePID;
  
  public IntakePivot() {
    m_intakePivot = new CANSparkMax(18, MotorType.kBrushless);
    m_intakeEncoder = m_intakePivot.getAbsoluteEncoder(Type.kDutyCycle);
    m_intakePID = m_intakePivot.getPIDController();
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  public void run()
}
*/