package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import edu.wpi.first.wpilibj.examples.ramsetecommand.Constants.DriveConstants;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.math.geometry.Pose2d;

public class DriveSubsystem extends SubsystemBase{
    private final MotorControllerGroup leftMotors =
            new MotorControllerGroup(new CANSparkMax(DriveConstants.kLefMotor1Port, MotorType.kBrushless),
                    new CANSparkMax(DriveConstants.kLefMotor2Port, MotorType.kBrushless));

    private final MotorControllerGroup rightMotors =
            new MotorControllerGroup(new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless),
                    new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless));

    private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotors, rightMotors);


    private final Encoder leftEncoder = new Encoder
            (DriveConstants.kLeftEncoderPorts[0],
                    DriveConstants.kLeftEncoderPorts[1]);

    private final Encoder rightEncoder = new Encoder
            (DriveConstants.kRightEncoderPorts[0],
                    DriveConstants.kRightEncoderPorts[1], true);

    private final Gyro gyro = new ADXRS450_Gyro();
    private final DifferentialDriveOdometry odometry;


    public DriveSubsystem(){
        rightMotors.setInverted(true);

        leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

        resetEncoders();
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    }
    @Override
    public void periodic(){
        odometry.update(gyro.getPosition2d, leftEncoder.getDistance(), rightEncoder.getDistance());
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    public void resetOdometry(Pose2d pose){
        resetEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    public void drive(double xSpeed, double rotation){
        diffDrive.arcadeDrive(xSpeed, rotation, true);
    }

    public void resetEncoders(){
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public double getAverageEncoderDistance(){
        return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
    }

    public Encoder getLeftEncoder(){
        return leftEncoder;
    }

    public Encoder getRightEncoder(){
        return rightEncoder;
    }

    public void setMaxOutput(double maxOutput){
        drive.setMaxOutput(maxOutput);
    }

    public void zeroHeading(){
        gyro.reset();
    }

    public double getHeading(){
        return gyro.getRotation2d().getDegrees();
    }

    public double getTurnRate(){
        return -gyro.getRate();
    }
}