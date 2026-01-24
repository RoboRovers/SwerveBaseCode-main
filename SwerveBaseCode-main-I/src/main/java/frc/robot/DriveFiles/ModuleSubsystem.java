package frc.robot.DriveFiles;


import java.util.Optional;
import com.ctre.phoenix6.CANBus;
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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.constants_Module;

import com.revrobotics.spark.config.SparkFlexConfig.Presets;;

public class ModuleSubsystem extends SubsystemBase
{


  public SparkMax driveMotor;
  public VelocityVoltage velocityRequest;
  public MotionMagicVelocityVoltage motionMagicRequest;
  // public TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  public NeutralModeValue neutralModeValue;
  public Slot0Configs driveGains;
  public FeedbackConfigs driveFeedbackConfigs;
  
  public LinearVelocity speedAt12Volts = Units.MetersPerSecond.of(4.55);
  public Current slipCurrent = edu.wpi.first.units.Units.Amps.of(120);
  public ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  public SparkMax steerMotor;
  public SparkClosedLoopController steerPIDController;
  public RelativeEncoder steerEncoder;
  public SparkBaseConfig steerGains;

  public Rotation2d absOffset;
  public CANcoder absoluteEncoder;
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


  //TODO HOW TO ZERO THE WHEELS
  /*
   * 1. First, connect to the robot and push this code using (shift -> f5)
   * 2. Second, after building the code, take a look at the dashboard which should show some basic data on the robot
   * 3. Third, turn one of the wheels and make sure its corresponding Relative encoder (such as "Back left RE Value") and Absolute encoder (such as "Back Left AE Value") are rotating in the same direction, meaning when you turn the wheel counterclockwise, both numbers should be going up positivly
   * 4. Fourth, using a bar of 1x2 aluminum or just something straight, align all wheels forward with the gear racks pointed inward
   * 5. Fifth, note down each of the modules Absolute Position and input them into the Constants file in the variables FL_OFFSET, FR_OFFSET, BL_OFFSET, BR_OFFSET
   * 6, Sixth, once you change the offset variables, go into the "MODULE" file, this one, and multiple the output of the "getABSPosition()" method variable by 360
   * 7. Seventh, Push the robot code again using (shift -> f5) and enable the robot once and then disable, then confirm by rotating each drive that both the Absolute encoder and the Relative encoder match and rotate at the same pace.
   * 8. Lastly, Head to the "Drive" Command file and uncomment the one line inside the "Initialize method" which activates the face forward method/algorithm
   */

  
  public void Module(int steerNum, int driveNum, boolean invertDrive, boolean invertSteer, int absoluteEncoderID, double absOffset, boolean absoluteReversed)
  {
    driveMotor = new SparkBaseConfig(driveNum);
    driveGains = new Slot0Configs().withKP(0.1).withKI(0).withKD(0.1).withKS(0.4).withKV(0.124);
    driveFeedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(constants_Module.DRIVE_GEAR_RATIO);
    neutralModeValue = NeutralModeValue.Brake;
    driveMotor.getConfigurator().apply(driveGains);
    driveMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false).withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(80));
    driveMotor.getConfigurator().apply(driveFeedbackConfigs, 5);
    driveMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(invertDrive?InvertedValue.Clockwise_Positive:InvertedValue.CounterClockwise_Positive).withNeutralMode(neutralModeValue));
  
    
    
    this.absoluteReversed = absoluteReversed;
    CANConfig = new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(absOffset).withAbsoluteSensorDiscontinuityPoint(0.5));
    
    absoluteEncoder = new CANcoder(absoluteEncoderID, new CANBus());
    absoluteEncoder.getConfigurator().apply(CANConfig);
    
    steerGains = new SparkMaxConfig()
    .apply(new ClosedLoopConfig().pidf(0.0225, 0.000001, 0.0, 0.0, ClosedLoopSlot.kSlot0).positionWrappingEnabled(true).positionWrappingInputRange(-179.9999999, 180));
    steerGains.encoder.positionConversionFactor(Constants.constants_Module.STEER_TO_DEGREES);
    steerGains.encoder.velocityConversionFactor(constants_Module.STEER__RPM_2_DEG_PER_SEC);
    steerGains.inverted(invertSteer);
    steerGains.idleMode(IdleMode.kBrake);
    steerGains.smartCurrentLimit(65);
    
    steerMotor = new SparkMax(steerNum, MotorType.kBrushless);
    steerEncoder = steerMotor.getEncoder();
    steerPIDController = steerMotor.getClosedLoopController();
    steerMotor.configure(steerGains, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    
    resetEncoders();
  }
  
  public void stop()
  {
    driveMotor.set(0);
    steerMotor.set(0);
  }
  
  public void resetEncoders() 
  {
    driveMotor.setPosition(0);
    steerEncoder.setPosition(0);
  }
  
//Drive Methods
  public double getDrivePosition()
  {
      return driveMotor.getPosition().getValueAsDouble() * constants_Module.DRIVE_ROT_2_METER;
  }
  public double getDriveVelocity()
  {
    return driveMotor.getVelocity().getValueAsDouble();
  }
  public void getUpToSpeed(double velocityMPS)
  {
    double rps = velocityMPS * constants_Module.DRIVE_MPS_2_RPS;
    driveMotor.setControl(new MotionMagicVelocityVoltage(rps));
  }
  
  
  
  //Steer Methods
  public double getPosition()
  {
    return steerEncoder.getPosition();
  }
  public double getABSPosition()
  {
    //TODO once you find all the offsets using this number that is printed out (Should range from -1 -> 1), multiply by 360 to convert to degrees
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
  public void setDesiredState(SwerveModuleState state) 
  {
    if (Math.abs(state.speedMetersPerSecond) < 0.01) {stop();return;}
    state.optimize(getModulePosition().angle);
    state.cosineScale(getModulePosition().angle);
    driveMotor.set(state.speedMetersPerSecond / Constants.constants_Drive.MAX_SPEED_METERS_PER_SEC);
    // getUpToSpeed(state.speedMetersPerSecond);
    steerPIDController.setReference(state.angle.getDegrees(), ControlType.kPosition);
  }

  public void wheelFaceForward() 
  {
    steerEncoder.setPosition(getABSPosition());
    try 
    {
      Thread.sleep(10);
      steerPIDController.setReference(0, ControlType.kPosition);
    } catch (Exception e){}
  }

  public void wheelFaceRight() 
  {
    steerEncoder.setPosition(getABSPosition());
    try 
    {
      Thread.sleep(10);
      steerPIDController.setReference(90, ControlType.kPosition);
    } catch (Exception e){}
  }
    
}
