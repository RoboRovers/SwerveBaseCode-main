package frc.robot.DriveFiles;

import java.util.Optional;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;

import frc.robot.DriveFiles.ModuleSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.studica.frc.AHRS;

public class SwerveSubsystem extends SubsystemBase{
    public final CommandXboxController opController = new CommandXboxController(OIConstants.kOPControllerPort);
    public boolean fieldOriented = false;
    public boolean hasReset = false;
    public boolean isRedAlliance;
    Optional<Alliance> alliance;
  
    
    public static Module frontLeftModule = new Module(Constants.constants_Drive.kFrontLeftTurningMotorPort, Constants.constants_Drive.kFrontLeftDriveMotorPort, Constants.constants_Drive.kFrontLeftDriveEncoderReversed, Constants.constants_Drive.kFrontLeftTurningEncoderReversed, Constants.constants_Drive.kFrontLeftDriveAbsoluteEncoderPort, Constants.constants_Drive.kBLDegrees, Constants.constants_Drive.kFrontLeftDriveAbsoluteEncoderReversed);
    public static Module frontRightModule = new Module(Constants.constants_Drive.kFrontRightTurningMotorPort, Constants.constants_Drive.kFrontRightDriveMotorPort, Constants.constants_Drive.kFrontRightDriveEncoderReversed, Constants.constants_Drive.kFrontRightTurningEncoderReversed, Constants.constants_Drive.kFrontRightDriveAbsoluteEncoderPort, Constants.constants_Drive.kBRDegrees, Constants.constants_Drive.kFrontRightDriveAbsoluteEncoderReversed);
    public static Module backLeftModule = new Module(Constants.constants_Drive.kBackLeftTurningMotorPort, Constants.constants_Drive.kBackLeftDriveMotorPort, Constants.constants_Drive.kBackLeftDriveEncoderReversed, Constants.constants_Drive.kBackLeftTurningEncoderReversed, Constants.constants_Drive.kBackLeftDriveAbsoluteEncoderPort, Constants.constants_Drive.kFLDegrees, Constants.constants_Drive.kBackLeftTurningEncoderReversed);
    public static Module backRightModule = new Module(Constants.constants_Drive.kBackRightTurningMotorPort, Constants.constants_Drive.kBackRightDriveMotorPort, Constants.constants_Drive.kBackRightDriveEncoderReversed, Constants.constants_Drive.kBackRightTurningEncoderReversed, Constants.constants_Drive.kBackRightDriveAbsoluteEncoderPort, Constants.constants_Drive.kFRDegrees, Constants.constants_Drive.kBackRightTurningEncoderReversed);
    

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(500);
                zeroHeading();
            } catch (Exception e) {}}).start();    
            
            alliance = getAlliance();
        }
        
        public void resetPositions(SwerveSubsystem FL, SwerveModule FR, SwerveModule BL, SwerveModule BR){
            FL.resetDriveEncoder();
            FR.resetDriveEncoder();
            BL.resetDriveEncoder();
            BR.resetDriveEncoder();
        }

            
    
        
    //gyro int and heading code
    private AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    public void zeroHeading() {
        gyro.reset();
        gyro.setAngleAdjustment(0);
    }
    public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }
    public Rotation2d geRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
        public boolean allianceCheck(){
        if (alliance.isPresent() && (alliance.get() == Alliance.Red)) {isRedAlliance = true;}else{isRedAlliance = false;}
        return isRedAlliance;
    }
    public Optional<Alliance> getAlliance(){
        return DriverStation.getAlliance();
    }

    //Odometer code
    public final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.constants_Drive.kDriveKinematics,
    geRotation2d(), getPositions(frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition()));
    
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }



    public final SwerveDriveOdometry autoOdometry = new SwerveDriveOdometry(Constants.constants_Drive.kDriveKinematics,
    geRotation2d(), getPositions(frontRightModule.getPosition(), frontLeftModule.getPosition(), backRightModule.getPosition(), backLeftModule.getPosition()));

    public void resetOdometry(Pose2d pose) {
        autoOdometry.resetPosition(geRotation2d(), getPositions(frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition()), pose);
        // hasReset = true;
    }

    public Pose2d getAutoPose()
    {
        return autoOdometry.getPoseMeters();
    }


    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeftModule.gState(), frontRightModule.gState(), backLeftModule.gState(), backRightModule.gState());
    }
    

    //stops all modules. Called when the command isn't being ran. So when an input isn't recieved
    public void stopModules() {
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }
    
    public void setModuleStates(ChassisSpeeds speeds){
        SwerveModuleState[] moduleStates = Constants.constants_Drive.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.constants_Drive.kPhysicalMaxSpeedMetersPerSecond);
        frontRightModule.setDesiredState(moduleStates[0]);
        frontLeftModule.setDesiredState(moduleStates[1]);
        backRightModule.setDesiredState(moduleStates[2]);
        backLeftModule.setDesiredState(moduleStates[3]);
    }
    
    public void driveRobotRelative(ChassisSpeeds speeds){
        // ChassisSpeeds.fromRobotRelativeSpeeds(speeds, geRotation2d());
        SwerveModuleState[] moduleStates = Constants.constants_Drive.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.constants_Drive.kPhysicalMaxSpeedMetersPerSecond);
        frontRightModule.setDesiredState(moduleStates[0]);
        frontLeftModule.setDesiredState(moduleStates[1]);
        backRightModule.setDesiredState(moduleStates[2]);
        backLeftModule.setDesiredState(moduleStates[3]);
    }
    
    
    //face forward method. Called once the bot is enabled
    public void faceAllFoward() {
        backRightModule.wheelFaceForward(Constants.constants_Drive.kBRDegrees);
        frontLeftModule.wheelFaceForward(Constants.constants_Drive.kFLDegrees);
        frontRightModule.wheelFaceForward(Constants.constants_Drive.kFRDegrees);
        backLeftModule.wheelFaceForward(Constants.constants_Drive.kBLDegrees);
        System.out.println("exacuted faceAll");
    }
    
    public Command resetWheels(){
        return runOnce(() -> {
                frontLeftModule.wheelFaceForward(Constants.constants_Drive.kFLDegrees);
                frontRightModule.wheelFaceForward(Constants.constants_Drive.kFRDegrees);
                backLeftModule.wheelFaceForward(Constants.constants_Drive.kBLDegrees);
                backRightModule.wheelFaceForward(Constants.constants_Drive.kBRDegrees);
    });}
        
    public Command fieldOrientedToggle(){
        return runOnce(() -> {fieldOriented = !fieldOriented;});
    }
    
    public SwerveModulePosition[] getPositions(SwerveModulePosition fl, SwerveModulePosition fr, SwerveModulePosition bl, SwerveModulePosition br){
        return new SwerveModulePosition[]{
            fl, fr, bl, br
        };
    }


    @Override
    public void periodic() {
            odometer.update(geRotation2d(), getPositions(
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition())
            );

            autoOdometry.update(geRotation2d(), getPositions(
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition())
            );



        
        
        //multiple debugging values are listed here. Names are self explanitory
        
                //Odometer and other gyro values
               SmartDashboard.putString("Robot Location", getAutoPose().getTranslation().toString());
               SmartDashboard.putNumber("Robot Heading", getHeading());
              
               //AE Degrees Reading
            //     SmartDashboard.putNumber("Back Left AE Value", backLeftModule.getAbsoluteEncoderDeg(Constants.DriveConstants.kBLDegrees));
            //     SmartDashboard.putNumber("Back Right AE Value", backRightModule.getAbsoluteEncoderDeg(Constants.DriveConstants.kBRDegrees));
            //     SmartDashboard.putNumber("Front Left AE Value", frontLeftModule.getAbsoluteEncoderDeg(Constants.DriveConstants.kFLDegrees));
            //     SmartDashboard.putNumber("Front Right AE Value", frontRightModule.getAbsoluteEncoderDeg(Constants.DriveConstants.kFRDegrees));
            // //   //RE Degrees Reading
            //     SmartDashboard.putNumber("Back left RE Value", backLeftModule.getSteerPosition());
            //     SmartDashboard.putNumber("Back Right RE Value", backRightModule.getSteerPosition());
            //     SmartDashboard.putNumber("Front left RE Value", frontLeftModule.getSteerPosition());
            //     SmartDashboard.putNumber("Front Right RE Value", frontRightModule.getSteerPosition());
            //  //RE Distance Reading
            //    SmartDashboard.putNumber("Front Left Drive Position", frontLeftModule.getDrivePosition());
            //    SmartDashboard.putNumber("Front Right Drive Position", frontRightModule.getDrivePosition());
            //    SmartDashboard.putNumber("Back Left Drive Position", backLeftModule.getDrivePosition());
            //    SmartDashboard.putNumber("Back Right Drive Position", backRightModule.getDrivePosition());
            

            //  flModPos = new SwerveModulePosition(frontLeftModule.getPositionMeters(), frontLeftModule.gState().angle);
            //  frModPos = new SwerveModulePosition(frontRightModule.getPositionMeters(), frontRightModule.gState().angle);
            //  blModPos = new SwerveModulePosition(backLeftModule.getPositionMeters(), backLeftModule.gState().angle);
            //  brModPos = new SwerveModulePosition(backRightModule.getPositionMeters(), backRightModule.gState().angle);
        }
}
