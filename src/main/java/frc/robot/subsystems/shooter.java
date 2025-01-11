package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
public class shooter extends SubsystemBase {
  
  SparkMax m_shooterMotor = new SparkMax(ShooterConstants.shooterMotorID, ShooterConstants.shooterMotorType);
  double speed;

    public void intake(){
      m_shooterMotor.set(0.5);
    }

    public void shoot(){
      m_shooterMotor.set(-0.5);
    }
   }