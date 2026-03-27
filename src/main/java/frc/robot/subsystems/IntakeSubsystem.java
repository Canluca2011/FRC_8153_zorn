package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    private final SparkMax intakeMotor;
    private final SparkMaxConfig config;

    public IntakeSubsystem() {
        intakeMotor = new SparkMax(IntakeConstants.kMotorId, MotorType.kBrushless);
        
        config = new SparkMaxConfig();
        config.smartCurrentLimit(IntakeConstants.kCurrentLimit); 
        
        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public void runForward() { intakeMotor.setVoltage(IntakeConstants.kIntakeSpeed); }
    public void runReverse() { intakeMotor.setVoltage(IntakeConstants.kReverseSpeed); }
    public void stop() { intakeMotor.setVoltage(0.0); }
}