package frc.robot.Subsystems.Drive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Util.Constants.ModuleConstants;


public class Module extends SubsystemBase{

    public SparkMax driveMotor;
    public SparkMax steerMotor;
    public CANcoder absoluteEncoder;

    public LinearVelocity speedAt12Volts = Units.MetersPerSecond.of(4.55);
    public Current slipCurrent = edu.wpi.first.units.Units.Amps.of(120);
    public ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    public NeutralModeValue neutralModeValue;
    public Slot0Configs driveValues;
    public FeedbackConfigs driveFeedbackConfigs;
    
    public VelocityVoltage velocityRequest;
    public MotionMagicVelocityVoltage motionMagicRequest;

    public SparkClosedLoopController steerPIDController;
    public RelativeEncoder steerEncoder;
    public SparkBaseConfig steerValues;

    public Rotation2d absOffset;
    public CANcoderConfiguration CANConfig;
    public boolean absoluteReversed;

     // Inputs from drive motor
    public StatusSignal<Angle> drivePosition;
    // public static Queue<Double> drivePositionQueue;
    public StatusSignal<AngularVelocity> driveVelocity;
    public StatusSignal<Voltage> driveAppliedVolts;
    public StatusSignal<Current> driveCurrent;

    // Inputs from turn motor
    public StatusSignal<Angle> steerAbsolutePosition;
    public StatusSignal<Angle> steerPosition;
    // public static Queue<Double> turnPositionQueue;
    public StatusSignal<AngularVelocity> turnVelocity;
    public StatusSignal<Voltage> turnAppliedVolts;
    public StatusSignal<Current> turnCurrent;


    // Voltage control requests
    public VoltageOut voltageRequest = new VoltageOut(0);
    // private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    public VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

    

    public Module(int steerMotorCANID, int driveMotorCANID, boolean invertDrive, boolean invertSteer, int absoluteEncoderCANID, double absOffset, boolean absoluteReversed)
    {
        driveMotor = new SparkMax(driveMotorCANID, MotorType.kBrushless);
        driveValues = new Slot0Configs().withKP(0.1).withKI(0).withKD(0.1).withKS(0.4).withKV(0.124);
        driveFeedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(ModuleConstants.kDriveMotorGearRatio);
        neutralModeValue = NeutralModeValue.Brake;
        
        steerMotor = new SparkMax(steerMotorCANID, MotorType.kBrushless);
        absoluteEncoder = new CANcoder(absoluteEncoderCANID);
        absoluteEncoder.getConfigurator().apply((CANConfig));

    }

    

}