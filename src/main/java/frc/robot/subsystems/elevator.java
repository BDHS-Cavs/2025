package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;

import edu.wpi.first.wpilibj.Solenoid;

public class elevator extends SubsystemBase {

  SparkMax m_elevatorMotor = new SparkMax(ElevatorConstants.elevatorMotorID, ElevatorConstants.elevatorMotorType); //elevator motor

  RelativeEncoder m_elevatorRelativeEncoder = m_elevatorMotor.getEncoder();//elevator motor encoder

  Solenoid m_latchSolenoid = new Solenoid(ElevatorConstants.elevatorPneumaticsModuleType, ElevatorConstants.latchSolenoidID);//locks elevator in place

  public void elevatorUp(){
    if(m_elevatorRelativeEncoder.getPosition() > -492) { //software limit switch
      m_latchSolenoid.set(false); //unlock
      m_elevatorMotor.set(0.5); //run motor
      }
    else {
      m_latchSolenoid.set(true); //lock
      m_elevatorMotor.set(0.0); //stop
    }
  }

  public void elevatorDown(){
    if(m_elevatorRelativeEncoder.getPosition() < 0) { //software limit switch
      m_latchSolenoid.set(false); //unlock
      m_elevatorMotor.set(-0.5); //run motor
      }
    else {
      m_latchSolenoid.set(true); //lock
      m_elevatorMotor.set(0.0); //stop
    }
  }

  public void elevatorStop(){
    m_elevatorMotor.set(0.0); //stop
  }

  }