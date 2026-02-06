package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//using rev motor for shooter 
public class shooter extends SubsystemBase
{
final SparkMax spark = new SparkMax(0, MotorType.kBrushless); //placeholder id for now


}
