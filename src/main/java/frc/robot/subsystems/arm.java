package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

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

  DoubleSolenoid m_latchSolenoid = new DoubleSolenoid(ArmConstants.armPneumaticsModuleType, ArmConstants.latchSolenoidForwardID, ArmConstants.latchSolenoidBackwardsID);

  public void armUp(){
    //TODO soft limit switch
    m_latchSolenoid.set(Value.kReverse); //unlock
    m_armMotor.set(0.5); //raise
  }

  public void armDown(){
    m_latchSolenoid.set(Value.kForward); //lock
    m_armMotor.set(-0.5); //lower
  }

  public void armStop(){
    m_armMotor.set(0.0);
  }

  }