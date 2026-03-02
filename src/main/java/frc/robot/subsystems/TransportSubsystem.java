package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import frc.robot.Constants.TransportConstants;

public class TransportSubsystem extends SubsystemBase {

    private final SparkMax transportMotorLeft;
    private final SparkMax transportMotorRight;

    private final SparkMaxConfig transportConfig;

    public TransportSubsystem() {
        transportMotorLeft = new SparkMax(TransportConstants.kLeftMotorId, MotorType.kBrushless);
        transportMotorRight = new SparkMax(TransportConstants.kRightMotorId, MotorType.kBrushless);

        transportConfig = new SparkMaxConfig();
        transportConfig.smartCurrentLimit(TransportConstants.kCurrentLimit); 
        
        transportMotorLeft.configure(transportConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        transportMotorRight.configure(transportConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public void runForward() {
        transportMotorLeft.set(-TransportConstants.kTransportSpeed);
        transportMotorRight.set(TransportConstants.kTransportSpeed);
    }  

    public void runReverse() {
        transportMotorLeft.set(TransportConstants.kTransportSpeed);
        transportMotorRight.set(-TransportConstants.kTransportSpeed);
    }

    public void stop() {
        transportMotorLeft.set(0.0); 
        transportMotorRight.set(0.0);
    }
}
