package frc.robot.Subsystems.Drive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Constants;
import frc.robot.Util.Constants.ModuleConstants;




public class Module extends SubsystemBase{


    public SparkMax driveMotor;
    public SparkMax steerMotor;
    public CANcoder absoluteEncoder;

    public LinearVelocity speedAt12Volts = Units.MetersPerSecond.of(4.55);
    public Current slipCurrent = edu.wpi.first.units.Units.Amps.of(120);
    public ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    public NeutralModeValue neutralModeValue;
    public SparkBaseConfig driveValues;
    public FeedbackConfigs driveFeedbackConfigs;
    
    public VelocityVoltage velocityRequest;
    public MotionMagicVelocityVoltage motionMagicRequest;

    public SparkClosedLoopController drivePIDController;
    public SparkClosedLoopController steerPIDController;
    public RelativeEncoder steerEncoder;
    public RelativeEncoder driveEncoder;
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
      driveEncoder = driveMotor.getEncoder();
      drivePIDController = driveMotor.getClosedLoopController();
      //steerMotor.configure(steerValues, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
      driveValues = new SparkMaxConfig()
      .apply(new ClosedLoopConfig().pidf(0.0225, 0.000001, 0.0, 0.0, ClosedLoopSlot.kSlot0).positionWrappingEnabled(true).positionWrappingInputRange(-179.9999999, 180));
      driveValues.encoder.positionConversionFactor(Constants.ModuleConstants.kDriveEncoderRot2Meter);
      driveValues.encoder.velocityConversionFactor(Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec);
      driveValues.inverted(invertDrive);
      driveValues.idleMode(IdleMode.kBrake);
      driveValues.smartCurrentLimit(65);
        
       
      steerMotor = new SparkMax(steerMotorCANID, MotorType.kBrushless);
      steerEncoder = steerMotor.getEncoder();
      steerPIDController = steerMotor.getClosedLoopController();
      //steerMotor.configure(steerValues, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
      steerValues = new SparkMaxConfig()
      .apply(new ClosedLoopConfig().pidf(0.0225, 0.000001, 0.0, 0.0, ClosedLoopSlot.kSlot0).positionWrappingEnabled(true).positionWrappingInputRange(-179.9999999, 180));
      steerValues.encoder.positionConversionFactor(Constants.ModuleConstants.kTurningConversionFactor2Deg);
      steerValues.encoder.velocityConversionFactor(Constants.ModuleConstants.kSteerEncoderRPM2DegPerSec);
      steerValues.inverted(invertSteer);
      steerValues.idleMode(IdleMode.kBrake);
      steerValues.smartCurrentLimit(65);

        this.absoluteReversed = absoluteReversed;
        CANConfig = new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(absOffset).withAbsoluteSensorDiscontinuityPoint(0.5));

        absoluteEncoder = new CANcoder(absoluteEncoderCANID);
        absoluteEncoder.getConfigurator().apply((CANConfig));


        resetEncoders();
    }

    public void stop()
    {
        driveMotor.set(0);
        steerMotor.set(0);

    
    }

    public void resetEncoders()
    {
        driveEncoder.setPosition(0);
        steerEncoder.setPosition(0);
    }

    public double getDrivePosition()
    {
        return driveEncoder.getPosition() * ModuleConstants.kDriveEncoderRot2Meter;
    }
    public double getDriveVelocity()
  {
    return driveEncoder.getVelocity();
  }
  public void getUpToSpeed(double velocityMPS)
  {
    double rps = velocityMPS * ModuleConstants.kDriveMPS2RPS;
    driveMotor.set(rps);
  }
  
   public void setDesiredState(SwerveModuleState state) 
  {
    if (Math.abs(state.speedMetersPerSecond) < 0.01) {stop();return;}
    state.optimize(getModulePosition().angle);
    state.cosineScale(getModulePosition().angle);
    driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    // getUpToSpeed(state.speedMetersPerSecond);
    steerPIDController.setSetpoint(state.angle.getDegrees(), ControlType.kPosition);
  }
  
  //Steer Methods
  public double getPosition()
  {
    return steerEncoder.getPosition();
  }
  public double getABSPosition()
  {
    
    double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble()*360; //  * 360 to convert to degrees
    return (angle  * (absoluteReversed ? -1 : 1) ) % 720;
  }
  public SwerveModuleState getModuleState()
  {
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getPosition()));
  }
  public SwerveModulePosition getModulePosition()
  {
    return new SwerveModulePosition(getDrivePosition(), getModuleState().angle);
  }

//ben is awesome
      
  //This is our setDesiredState alg. Takes the current state and the desired state shown by the controller and points the wheels to that location


  public void wheelFaceForward() 
  {
    steerEncoder.setPosition(getABSPosition());
    try 
    {
      Thread.sleep(10);
      steerPIDController.setSetpoint(0, ControlType.kPosition);
    } catch (Exception e){}
  }

  public void wheelFaceRight() 
  {
    steerEncoder.setPosition(getABSPosition());
    try 
    {
      Thread.sleep(10);
      steerPIDController.setSetpoint(90, ControlType.kPosition);
    } catch (Exception e){}
  }
    
}


    

