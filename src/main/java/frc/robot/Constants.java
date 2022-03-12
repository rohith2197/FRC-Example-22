// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants{
        public static final int kLefMotor1Port = 3;
        public static final int kLefMotor2Port = 5;

        public static final int kRightMotor1Port = 4;
        public static final int kRightMotor2Port = 8;

        public static final int kShooter1 = 11;
        //constants for each motor (k shooter is for the shooter)

        public static final double ksVolts = 0.4;
        public static final double kvVoltSecondsPerMeter = 1.5;
        public static final double kaVoltSecondsSquaredPerMeter = 0.3;

        public static final double kPDriveVel = 5.5;
        //constants for feedforward

        public static final double kTrackWidthMeters = 0.64;
        public static final DifferentialDriveKinematics kDriveKinematics
                = new DifferentialDriveKinematics(kTrackWidthMeters);
        //constants for trackwidth

        public static final double kMaxSpeed = 4;
        //meters per second
        public static final double kMaxAcceleration = 4;
        //meters per second squared

        //constants for max speed/acceleration

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        //constants for ramsete controller

        public static final int[] kLeftEncoderPorts = {6, 7};
        public static final int[] kRightEncoderPorts = {9, 10};

        public static final double kEncoderDistancePerPulse = 0.2;

    }
    public static final class OIConstants{
        public static final int controller = 0;
    }
}
