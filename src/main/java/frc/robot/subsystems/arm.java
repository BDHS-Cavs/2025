package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class arm extends SubsystemBase {

  Spark m_armExtensionMotor = new Spark(ArmConstants.armExtensionMotorID); //pivot motor
  Spark m_armPivotMotor = new Spark(ArmConstants.armPivotMotorID); //extension motor

//set motor to brake
  public void armUp(){
      m_armPivotMotor.set(0.1); //raise 
  }

  public void armDown(){
      m_armPivotMotor.set(-0.1); //lower
  }

  public void armStop(){
    m_armPivotMotor.set(0.0); //stop
  }

  }