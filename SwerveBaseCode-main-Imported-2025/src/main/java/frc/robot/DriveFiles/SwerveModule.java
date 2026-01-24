package frc.robot.DriveFiles;

import frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.*;



public class SwerveModule extends SubsystemBase{
    
  //initalize all variables
    public static CANSparkMax steerMotor;
    public CANSparkMax driveMotor;
    public final SparkPIDController turningPidController;


    private static final double RAMP_RATE = 0.85;//1.5;
    public RelativeEncoder driveMotorEncoder;
    public RelativeEncoder steerMotorEncoder;
    public CANcoder absoluteEncoder;
    private boolean absoluteEncoderReversed;
    public final SparkPIDController drivePidController;



  //New Swerve Module start
  public SwerveModule(int steerNum, int driveNum, boolean invertDrive, boolean invertSteer, int absoluteEncoderId,
  double absoluteEncoderOffsetRad, Boolean absoluteEncoderReversed) {

    //Steer + Drive Motor Config
    driveMotor = new CANSparkMax(driveNum, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    driveMotor.restoreFactoryDefaults();
    driveMotor.setInverted(invertDrive);
    driveMotor.setOpenLoopRampRate(RAMP_RATE);
    driveMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
     
    steerMotor = new CANSparkMax(steerNum, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    steerMotor.restoreFactoryDefaults();
    steerMotor.setInverted(invertSteer);
    steerMotor.setOpenLoopRampRate(RAMP_RATE);
    steerMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);


    //Steer + Driving PID Controllers
    turningPidController = steerMotor.getPIDController();
    turningPidController.setP(Constants.ModuleConstants.kPTurning);
    turningPidController.setI(Constants.ModuleConstants.kITurning);
    turningPidController.setD(Constants.ModuleConstants.kITurning);
    turningPidController.setPositionPIDWrappingEnabled(true);
    turningPidController.setPositionPIDWrappingMaxInput(1080); 
    turningPidController.setPositionPIDWrappingMinInput(720);
    
    drivePidController = driveMotor.getPIDController();
    drivePidController.setP(0.95);
    
    //Absolute Encoder
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new CANcoder(absoluteEncoderId);
    
    
    //Steer + Drive Motor Encoder
    driveMotorEncoder = driveMotor.getEncoder();
    driveMotorEncoder.setPositionConversionFactor(1/22.5);
    driveMotorEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec);

    steerMotorEncoder = steerMotor.getEncoder();
    steerMotorEncoder.setPositionConversionFactor(Constants.ModuleConstants.kTurningConversionFactor2Deg);
    steerMotorEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kTurningEncoderRPM2DegPerSec);

 //reset encoders after init phase
    resetDrive();
  }

public static SparkPIDController getPIDController() {
  return steerMotor.getPIDController();
}
public void resetDrive() {
  driveMotorEncoder.setPosition(0);
  steerMotorEncoder.setPosition(0);
}
public void resetDriveEncoder() {
  driveMotorEncoder.setPosition(0);
}
//stop method that stops the motors when the stick/s are within the deadzone < 0.01
public void stop() {
  driveMotor.set(0);
  steerMotor.set(0);
}

  public double getAbsoluteEncoderDeg(double AEOffset) {
    double angle = absoluteEncoder.getPosition().getValueAsDouble();
    angle *= 360;
    return (angle  * (absoluteEncoderReversed ? -1 : 1) - AEOffset) % 720;
  }
  
  //Motor calls
  public double getDrivePosition() {
    return driveMotorEncoder.getPosition();
  }
  public double getDriveVelocity() {
    return driveMotorEncoder.getVelocity();
  }
  public double getSteerPosition() {
     return Math.abs(steerMotorEncoder.getPosition() % 720);
  }
  public double getSteerVelocity() {
    return steerMotorEncoder.getVelocity();
  }
  public double getPositionMeters() {
    return driveMotorEncoder.getPosition();
  }
  
 
  
//Creating the current state of the modules. A drive velo and an angle are needed. We use an off set of -90 for the angle
public SwerveModuleState gState() {
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(steerMotorEncoder.getPosition()));
}
public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotorEncoder.getPosition(), Rotation2d.fromDegrees(steerMotorEncoder.getPosition()));
}

//This is our setDesiredState alg. Takes the current state and the desired state shown by the controller and points the wheels to that 
//location
public void setDesiredState(SwerveModuleState state) {
  if (Math.abs(state.speedMetersPerSecond) < 0.01) {stop();return;}
  state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(gState().angle.getDegrees()));
  driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
  turningPidController.setReference(state.angle.getDegrees(), com.revrobotics.CANSparkBase.ControlType.kPosition);
  
}

public void wheelFaceForward(double AEOffset) {
  steerMotorEncoder.setPosition(getAbsoluteEncoderDeg(AEOffset));
  try{Thread.sleep(10);
    turningPidController.setReference(0, com.revrobotics.CANSparkBase.ControlType.kPosition);
  }catch (Exception e) {}}


}