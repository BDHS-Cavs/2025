package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class arm extends SubsystemBase {

  SparkMax m_armExtensionMotor = new SparkMax(ArmConstants.armExtensionMotorID, ArmConstants.armMotorType); //pivot motor
  SparkMax m_armPivotMotor = new SparkMax(ArmConstants.armPivotMotorID,ArmConstants.armMotorType); //extension motor

//TODO set motor to brake? -- with sparkmax i think rev hardware client does it

  public arm(){ //init
  }

  public void periodic(){ //periodic
  }

  public void armUp(){
      m_armPivotMotor.set(1.0); //raise 
  }

  public void armDown(){
      m_armPivotMotor.set(-1.0); //lower
  }

  public void armPivotStop(){
    m_armPivotMotor.set(0.0); //stop
  }

  public void armExtend(){
    m_armExtensionMotor.set(1.0); //raise 
  }

  public void armRetract(){
    m_armExtensionMotor.set(-1.0); //lower
  }

  public void armExtensionStop(){
    m_armExtensionMotor.set(0.0); //stop
  }

  }