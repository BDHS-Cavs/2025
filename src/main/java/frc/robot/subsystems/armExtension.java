package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class armExtension extends SubsystemBase {

  SparkMax m_armExtensionMotor = new SparkMax(ArmConstants.armExtensionMotorID, ArmConstants.armMotorType); //extension motor

  AnalogInput m_armExtensionLimitSwitch = new AnalogInput(ArmConstants.armExtensionLimitID);
  AnalogInput m_armRetractionLimitSwitch = new AnalogInput(ArmConstants.armRetractionLimitID);

//TODO set motor to brake? -- with sparkmax i think rev hardware client does it

  public armExtension(){ //init
  }

  public void periodic(){ //periodic
    SmartDashboard.putNumber("Arm Extension Limit", m_armExtensionLimitSwitch.getValue());
  }

  public void armExtend(){
    if(m_armExtensionLimitSwitch.getValue() < 10) { //physical limit switch
      m_armExtensionMotor.set(1.0); //extend
      }
    else {
      m_armExtensionMotor.set(0.0); //stop extension bc limit switch
    }
  }

  public void armRetract(){
    if(m_armRetractionLimitSwitch.getValue() > 10) { //physical limit switch
      m_armExtensionMotor.set(-1.0); //retract
      }
    else {
      m_armExtensionMotor.set(0.0); //stop retraction bc limit switch
    }
  }

  public void armExtensionStop(){
    m_armExtensionMotor.set(0.0); //stop extension motor
  }

  }