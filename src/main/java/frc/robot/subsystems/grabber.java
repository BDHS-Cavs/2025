package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.GrabberConstants;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class grabber extends SubsystemBase {

  Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);
  DoubleSolenoid m_latchSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
  DoubleSolenoid m_wristSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);

  public void compressorEnable(){
    m_compressor.enableAnalog(GrabberConstants.CompressorMin, GrabberConstants.CompressorMax);
  }

  public void grabberOpen(){
    m_latchSolenoid.set(Value.kForward);
  }

  public void grabberClose(){
    m_latchSolenoid.set(Value.kReverse);
  }

  public void compressorDisable(){
    m_compressor.disable();
  }
  }