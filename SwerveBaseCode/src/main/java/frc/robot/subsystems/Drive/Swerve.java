package frc.robot.Subsystems.Drive;

import java.util.Optional;

import com.studica.frc.AHRS;

// import com.studica.frc.AHRS;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Constants.DriveConstants;
import frc.robot.Util.RobotMap.Map_Controller;
import frc.robot.Util.RobotMap.Map_DriveTrain;

public class Swerve extends SubsystemBase{
    public boolean fieldOriented = false;
    public boolean hasReset = false;
    public boolean isRedAlliance;
    Optional<Alliance> alliance;
  
    
    public static Module frontLeftModule = new Module(Map_DriveTrain.Front_Left_CANCoder, Map_DriveTrain.Front_Left_Drive, DriveConstants.kFrontLeftDriveEncoderReversed, DriveConstants.kFrontLeftSteerEncoderReversed, Map_DriveTrain.Front_Left_CANCoder, DriveConstants.kFLOffset, DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
    public static Module frontRightModule = new Module(Map_DriveTrain.Front_Right_CANCoder, Map_DriveTrain.Front_Right_Drive, DriveConstants.kFrontRightDriveEncoderReversed, DriveConstants.kFrontRightSteerEncoderReversed, Map_DriveTrain.Front_Right_CANCoder, DriveConstants.kFROffset, DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
    public static Module backLeftModule = new Module(Map_DriveTrain.Back_Left_CANCoder, Map_DriveTrain.Back_Left_Drive, DriveConstants.kBackLeftDriveEncoderReversed, DriveConstants.kBackLeftSteerEncoderReversed, Map_DriveTrain.Back_Left_CANCoder, DriveConstants.kBLOffset, DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
    public static Module backRightModule = new Module(Map_DriveTrain.Back_Right_CANCoder, Map_DriveTrain.Back_Right_Drive, DriveConstants.kBackRightDriveEncoderReversed, DriveConstants.kBackRightSteerEncoderReversed, Map_DriveTrain.Back_Right_CANCoder, DriveConstants.kBROffset, DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
    
    
    public Swerve() 
    {
        new Thread(() -> {try
        {
            Thread.sleep(500);
            zeroHeading();
        } catch (Exception e) {}}).start();    
        // alliance = getAlliance();
    }
        
    //gyro int and heading code
    private AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    
    public void zeroHeading() 
    {
        gyro.reset();
        gyro.setAngleAdjustment(0);
    }
    
    public void setAngleAdjustment(double angle)
    {
        gyro.setAngleAdjustment(angle+90);
    }
    
    public double getHeading() 
    {
        return Math.IEEEremainder(-gyro.getAngle(), 360); //TODO Should be counter-clockwise as positive rotation
    }
    
    public Rotation2d getRotation2d() 
    {
        return Rotation2d.fromDegrees(getHeading());
    }
    
    public boolean allianceCheck() 
    {
        // if (alliance.isPresent() && (alliance.get() == Alliance.Red)) {isRedAlliance = true;}else{isRedAlliance = false;}
        // return isRedAlliance;
        var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
    }

    //Odometer code
    public final SwerveDriveOdometry odometer = new SwerveDriveOdometry
    (
        DriveConstants.kDriveKinematics,
        getRotation2d(),
        getModulePositions()
    );
    
    public Pose2d getPose() 
    {
        return odometer.getPoseMeters().rotateBy(Rotation2d.fromDegrees(180));
    }
    
    public void resetOdometry(Pose2d pose) 
    {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose.rotateBy(Rotation2d.fromDegrees(180)));
    }
    
    public static SwerveModulePosition[] getModulePositions()
    {
        return new SwerveModulePosition[]{
            frontLeftModule.getModulePosition(),
            frontRightModule.getModulePosition(),
            backLeftModule.getModulePosition(),
            backRightModule.getModulePosition()
        };
    }
    
    public ChassisSpeeds getRobotRelativeSpeeds()
    {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
            frontLeftModule.getModuleState(),
            frontRightModule.getModuleState(),
            backLeftModule.getModuleState(),
            backRightModule.getModuleState());
    }

    public void setModuleStates(ChassisSpeeds speeds){
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
        frontLeftModule.setDesiredState(moduleStates[0]);
        frontRightModule.setDesiredState(moduleStates[1]);
        backLeftModule.setDesiredState(moduleStates[2]);
        backRightModule.setDesiredState(moduleStates[3]);
        // SmartDashboard.putNumber("FL module desired Degrees", moduleStates[1].angle.getDegrees());
    }
    
    //face forward method. Called once the bot is enabled
    public void faceAllFoward() {
        frontLeftModule.wheelFaceForward();
        frontRightModule.wheelFaceForward();
        backLeftModule.wheelFaceForward();
        backRightModule.wheelFaceForward();
        System.out.println("exacuted faceAll Forward");
    }

    //face right method. Called once the bot is enabled
    public void faceAllRight() {
        frontLeftModule.wheelFaceRight();
        frontRightModule.wheelFaceRight();
        backLeftModule.wheelFaceRight();
        backRightModule.wheelFaceRight();
        System.out.println("exacuted faceAll Right");
    }
    
    public Command resetWheels(){
        return runOnce(() -> {
            faceAllFoward();
        });
    }

    //stops all modules. Called when the command isn't being ran. So when an input isn't recieved
    public void stopModules() {
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }
        
    public Command fieldOrientedToggle(){
        return runOnce(() -> {fieldOriented = !fieldOriented;});
    }
    
    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());

        //multiple debugging values are listed here. Names are self explanitory
        
        //Odometer and other gyro values
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putBoolean("fieldOriented", fieldOriented);
        
        //AE Degrees Reading
        SmartDashboard.putNumber("Front Left AE Value", frontLeftModule.getABSPosition());
        SmartDashboard.putNumber("Front Right AE Value", frontRightModule.getABSPosition());
        SmartDashboard.putNumber("Back Left AE Value", backLeftModule.getABSPosition());
        SmartDashboard.putNumber("Back Right AE Value", backRightModule.getABSPosition());

        //RE Degrees Reading
        SmartDashboard.putNumber("Front left RE Value", frontLeftModule.getModulePosition().angle.getDegrees());
        SmartDashboard.putNumber("Front Right RE Value", frontRightModule.getModulePosition().angle.getDegrees());
        SmartDashboard.putNumber("Back left RE Value", backLeftModule.getModulePosition().angle.getDegrees());
        SmartDashboard.putNumber("Back Right RE Value", backRightModule.getModulePosition().angle.getDegrees());
        //RE Distance Reading
        SmartDashboard.putNumber("Front Left Drive Position", frontLeftModule.getDrivePosition());
        SmartDashboard.putNumber("Front Right Drive Position", frontRightModule.getDrivePosition());
        SmartDashboard.putNumber("Back Left Drive Position", backLeftModule.getDrivePosition());
        SmartDashboard.putNumber("Back Right Drive Position", backRightModule.getDrivePosition());

        /*SmartDashboard.putNumber("FL Drive Temp", frontLeftModule.driveMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("FR Drive Temp", frontRightModule.driveMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("BL Drive Temp", backLeftModule.driveMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("BR Drive Temp", backRightModule.driveMotor.getDeviceTemp().getValueAsDouble());*/
        
    

    //  flModPos = new SwerveModulePosition(frontLeftModule.getDrivePosition(), frontLeftModule.getSteerState().angle);
    //  frModPos = new SwerveModulePosition(frontRightModule.getDrivePosition(), frontRightModule.getSteerState().angle);
    //  blModPos = new SwerveModulePosition(backLeftModule.getDrivePosition(), backLeftModule.getSteerState().angle);
    //  brModPos = new SwerveModulePosition(backRightModule.getDrivePosition(), backRightModule.getSteerState().angle);

        }
}