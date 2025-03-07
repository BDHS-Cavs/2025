package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class arm extends SubsystemBase {

  Spark m_armExtensionMotor = new Spark(ArmConstants.armExtensionMotorID); //pivot motor
  Spark m_armPivotMotor = new Spark(ArmConstants.armPivotMotorID); //extension motor

//TODO set motor to brake?

  public void periodic(){
    SmartDashboard.putNumber("Spark 0 Voltage", m_armExtensionMotor.getVoltage());
    SmartDashboard.putNumber("Spark 1 Voltage", m_armPivotMotor.getVoltage());
    SmartDashboard.putString("Spark 0 Description", m_armExtensionMotor.getDescription());
    SmartDashboard.putString("Spark 1 Description", m_armPivotMotor.getDescription());
  }
  public void armUp(){
      m_armPivotMotor.set(0.1); //raise 
  }

  public void armDown(){
      m_armPivotMotor.set(-0.1); //lower
  }

  public void armPivotStop(){
    m_armPivotMotor.set(0.0); //stop
  }

  public void armExtend(){
    m_armExtensionMotor.set(0.1); //raise 
  }

  public void armRetract(){
    m_armExtensionMotor.set(-0.1); //lower
  }

  public void armExtensionStop(){
    m_armExtensionMotor.set(0.0); //stop
  }

  }