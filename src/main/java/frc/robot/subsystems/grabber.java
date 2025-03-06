package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.PneumaticConstants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class grabber extends SubsystemBase {

  SparkMax m_grabberMotor = new SparkMax(GrabberConstants.grabberMotorID, GrabberConstants.grabberMotorType);

  PneumaticHub m_pneumaticHub = new PneumaticHub(PneumaticConstants.pneumaticHubID);

  DoubleSolenoid m_wristSolenoid = new DoubleSolenoid(PneumaticConstants.pneumaticHubID, PneumaticConstants.pneumaticHubModuleType, GrabberConstants.wristSolenoidForwardID, GrabberConstants.wristSolenoidBackwardID);

  public void periodic() {
    SmartDashboard.putNumber("compressor analog voltage", m_pneumaticHub.getCompressorCurrent());
  }

  public void compressorEnable(){
    m_pneumaticHub.enableCompressorAnalog(PneumaticConstants.compressorMin, PneumaticConstants.compressorMax);
  }

  public void grabberOut(){
    m_grabberMotor.set(1.0);
  }

  public void grabberIn(){
    m_grabberMotor.set(-0.5);
  }

  public void grabberStop(){
    m_grabberMotor.set(0.0);
  }

  public void compressorDisable(){
    m_pneumaticHub.disableCompressor();
  }

  public void wristRotate(){
    m_wristSolenoid.set(Value.kForward);
  }

  public void wristRotateOther(){
    m_wristSolenoid.set(Value.kReverse);
  }
  }