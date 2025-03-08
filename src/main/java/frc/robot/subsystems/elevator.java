package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PneumaticConstants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class elevator extends SubsystemBase {

  SparkMax m_elevatorMotor = new SparkMax(ElevatorConstants.elevatorMotorID, ElevatorConstants.elevatorMotorType); //elevator motor

  RelativeEncoder m_elevatorRelativeEncoder = m_elevatorMotor.getEncoder();//elevator motor encoder

  Solenoid m_latchSolenoid = new Solenoid(PneumaticConstants.pneumaticHubID, PneumaticConstants.pneumaticHubModuleType, ElevatorConstants.latchSolenoidID);//locks elevator in place

  public elevator(){ //init
  }

  public void periodic(){ //periodic
    SmartDashboard.putNumber("Elevator Encoder", m_elevatorRelativeEncoder.getPosition());
  }

  public void elevatorUp(){
    if(m_elevatorRelativeEncoder.getPosition() > -280) { //software limit switch
      m_latchSolenoid.set(true); //unlock
      m_elevatorMotor.set(-1.0); //run motor //UP IS NEGATIVE DOWN IS POSITIVE
      }
    else {
      m_latchSolenoid.set(false); //lock
      m_elevatorMotor.set(0.0); //stop
    }
  }

  public void elevatorDown(){
    if(m_elevatorRelativeEncoder.getPosition() < 0) { //software limit switch
      m_latchSolenoid.set(true); //unlock
      m_elevatorMotor.set(0.7); //run motor //UP IS NEGATIVE DOWN IS POSITIVE
      }
    else {
      m_latchSolenoid.set(false); //lock
      m_elevatorMotor.set(0.0); //stop
    }
  }

  public void elevatorStop(){
    m_elevatorMotor.set(0.0); //stop
  }

  public void elevatorEncoderReset(){
    m_elevatorRelativeEncoder.setPosition(0); //reset
  }

  }