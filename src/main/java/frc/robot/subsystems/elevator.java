package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class elevator extends SubsystemBase {

  SparkMax m_elevatorMotor = new SparkMax(ElevatorConstants.elevatorMotorID, ElevatorConstants.elevatorMotorType);

  RelativeEncoder m_elevatorRelativeEncoder = m_elevatorMotor.getEncoder();

  public void elevatorUp(){
    m_elevatorMotor.set(0.5);
  }

  public void elevatorDown(){
    m_elevatorMotor.set(-0.5);
  }

  public void elevatorStop(){
    m_elevatorMotor.set(0.0);
  }

  }