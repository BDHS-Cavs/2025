package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.PneumaticConstants;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class grabber extends SubsystemBase {

  SparkMax m_grabberMotor = new SparkMax(GrabberConstants.grabberMotorID, GrabberConstants.grabberMotorType);

  PneumaticHub m_pneumaticHub = new PneumaticHub(PneumaticConstants.pneumaticHubID);

  //Compressor m_compressor = new Compressor(GrabberConstants.grabberPneumaticsModuleType);

  DoubleSolenoid m_wristSolenoid = new DoubleSolenoid(PneumaticConstants.pneumaticHubID, PneumaticConstants.pneumaticHubModuleType, GrabberConstants.wristSolenoidForwardID, GrabberConstants.wristSolenoidBackwardID);

  int rotatestatus = 0;

  public void periodic() {
    //SmartDashboard.putNumber("compressor analog voltage", m_compressor.getAnalogVoltage());
  }

  public void compressorEnable(){
    //m_compressor.enableAnalog(GrabberConstants.compressorMin, GrabberConstants.compressorMax);
    m_pneumaticHub.enableCompressorAnalog(GrabberConstants.compressorMin, GrabberConstants.compressorMax);
  }

  public void grabberOut(){
    m_grabberMotor.set(0.2);
  }

  public void grabberIn(){
    m_grabberMotor.set(-0.2);
  }

  public void grabberStop(){
    m_grabberMotor.set(0.0);
  }

  public void compressorDisable(){
    //m_compressor.disable();
    m_pneumaticHub.disableCompressor();
  }

  public void wristRotate(){
    if (rotatestatus == 0) {
      m_wristSolenoid.set(Value.kForward);
      rotatestatus = 1;
    } else if(rotatestatus == 1) {
      m_wristSolenoid.set(Value.kReverse);
      rotatestatus = 0;
    }
  }
  }