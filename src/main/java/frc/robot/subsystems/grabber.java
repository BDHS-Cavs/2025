package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.GrabberConstants;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class grabber extends SubsystemBase {

  SparkMax m_grabberMotor = new SparkMax(GrabberConstants.grabberMotorID, GrabberConstants.grabberMotorType);

  Compressor m_compressor = new Compressor(GrabberConstants.grabberPneumaticsModuleType);//TODO min max?

  DoubleSolenoid m_wristSolenoid = new DoubleSolenoid(GrabberConstants.grabberPneumaticsModuleType, GrabberConstants.wristSolenoidForwardID, GrabberConstants.wristSolenoidForwardID);

  public void compressorEnable(){
    m_compressor.enableAnalog(GrabberConstants.compressorMin, GrabberConstants.compressorMax);
  }

  public void grabberOut(){
    m_grabberMotor.set(0.5);
  }

  public void grabberIn(){
    m_grabberMotor.set(-0.5);
  }

  public void grabberStop(){
    m_grabberMotor.set(0.0);
  }

  public void compressorDisable(){
    m_compressor.disable();
  }
//2 functions on a buttonpress to chnage the reverse/forward on wrist solenoid
  }