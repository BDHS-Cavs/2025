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

public class wrist extends SubsystemBase {
  
  DoubleSolenoid m_wristSolenoid = new DoubleSolenoid(PneumaticConstants.pneumaticHubID, PneumaticConstants.pneumaticHubModuleType, GrabberConstants.wristSolenoidForwardID, GrabberConstants.wristSolenoidBackwardID);

  public wrist(){ //init
  }

  public void periodic() { //periodic
  }

  public void wristRotate(){
    m_wristSolenoid.set(Value.kForward);
  }

  public void wristRotateOther(){
    m_wristSolenoid.set(Value.kReverse);
  }
  }