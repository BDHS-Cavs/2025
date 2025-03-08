package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class armPivot extends SubsystemBase {

  SparkMax m_armPivot1Motor = new SparkMax(ArmConstants.armPivotMotor1ID,ArmConstants.armMotorType); //pivot motor 1
  SparkMax m_armPivot2Motor = new SparkMax(ArmConstants.armPivotMotor2ID,ArmConstants.armMotorType); //pivot motor 1

//TODO set motor to brake? -- with sparkmax i think rev hardware client does it

  public armPivot(){ //init
  }

  public void periodic(){ //periodic
  }

  public void armUp(){
      m_armPivot1Motor.set(0.7); //raise 1
      m_armPivot2Motor.set(-0.7); //raise 2
  }

  public void armDown(){
      m_armPivot1Motor.set(-0.7); //lower 1
      m_armPivot2Motor.set(0.7); //lower 2
  }

  public void armPivotStop(){
    m_armPivot1Motor.set(0.0); //stop pivot motor 1
    m_armPivot2Motor.set(0.0); //stop pivot motor 2
  }

  }