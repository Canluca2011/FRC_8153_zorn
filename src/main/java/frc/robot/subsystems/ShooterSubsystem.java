package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    
    private final SparkMax shooterMotorLeft;
    private final SparkMax shooterMotorRight;

    private final SparkMaxConfig shooterConfig;

    public ShooterSubsystem() {
        shooterMotorLeft = new SparkMax(ShooterConstants.kLeftMotorId, MotorType.kBrushless);
        shooterMotorRight = new SparkMax(ShooterConstants.kRightMotorId, MotorType.kBrushless);

        shooterConfig = new SparkMaxConfig();
        shooterConfig.smartCurrentLimit(ShooterConstants.kCurrentLimit); 
        
        shooterMotorLeft.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotorRight.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public void runForward() {
        shooterMotorLeft.setVoltage(ShooterConstants.kShootSpeed);
        shooterMotorRight.setVoltage(-ShooterConstants.kShootSpeed); // Inverted relative to left
    }


    public void stop() {
        shooterMotorLeft.setVoltage(0.0);
        shooterMotorRight.setVoltage(0.0);
    }

}
