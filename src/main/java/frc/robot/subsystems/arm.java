package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class arm extends SubsystemBase {

  SparkMax m_armMotor = new SparkMax(ArmConstants.armMotorID, ArmConstants.armMotorType);

  RelativeEncoder m_armRelativeEncoder = m_armMotor.getEncoder();

  DoubleSolenoid m_latchSolenoid = new DoubleSolenoid(ArmConstants.armPneumaticsModuleType, ArmConstants.latchSolenoidForwardID, ArmConstants.latchSolenoidBackwardsID);

  public void armUp(){
    if(m_armRelativeEncoder.getPosition() > -492) {
      m_latchSolenoid.set(Value.kReverse); //unlock
      m_armMotor.set(0.5); //raise 
    }
    else {
      m_latchSolenoid.set(Value.kForward); //lock
      m_armMotor.set(0.0); //stop 
    }
  }

  public void armDown(){
    if(m_armRelativeEncoder.getPosition() < 0) {
      m_latchSolenoid.set(Value.kReverse); //unlock
      m_armMotor.set(-0.5); //lower
    }
    else {
      m_latchSolenoid.set(Value.kForward); //lock
      m_armMotor.set(0.0); //stop
    }
  }

  public void armStop(){
    m_armMotor.set(0.0);
  }

  }