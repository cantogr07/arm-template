// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.team6014.lib.math.Conversions;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.team6014.lib.math.Gearbox;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class ArmConstants {

    public static final int MOTOR_ID = 20;
    public static final int BORE_ID = 1;

    public static final Gearbox gearRatio = new Gearbox(1, 115.77);
    public static final boolean ARM_MOTOR_INVERTED = false;

    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

    /** unit: rev/s */
    public static final double ARM_VELOCITY = 200;
    /** unit: rev/s^2 */
    public static final double ARM_ACCyELERATION = 170;

    public static final double kP = 1.2;
    public static final double kD = 0.0;
    public static final double kI = 0;
    public static final double kS = 0.14;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kF = 0.4;

    /** unit: rotations */
    // before arm broke down
    public static final double POSITION_OFFSET_OLD = Conversions.degreesToRevolutions(13.34);
    // final offset used in the regs
    public static final double POSITION_OFFSET = Conversions
        .degreesToRevolutions(80 - 4 + 2 + 2);

    /** unit: rotations */
    public static final double ANGLE_TOLERANCE = Conversions.degreesToRevolutions(0.5);

    /* Arm angles for setpoints (relative to the zero of Bore) */
    /** unit: degrees */
    public static final double ZERO = 0;
    /** unit: degrees */
    public static final double INTAKE = 10; // 10.5
    /** unit: degrees */
    public static final double SPEAKER_LONG = 54;
    /** unit: degrees */
    public static final double SPEAKER_SHORT = 32; // 22.5 31.5 36
    /** unit: degrees */
    public static final double FROM_INTAKE = 75;
    /** unit: degrees */
    public static final double AMP = 110;
    /** unit degrees */
    public static final double CLIMB = 45;
    /** unit: degrees */
    public static final double NOTE_PASS = 25;

    /** unit: degrees */
    public static final double LAST_RESORT_ANGLE_CUTOFF = 120;

    // interpolation - currently using lookup tables
    // quadratic fit
    public static final double COEFFICIENT_QUADRATIC = -1.770;
    public static final double COEFFICIENT_LINEAR = 17.05;
    public static final double COEFFICIENT_CONSTANT = 21.47;
    public static final boolean IS_ON_FIELD = true;

    // gaussian fit
    public static final double G_COEFFICENT_A = -24.19;
    public static final double G_COEFFICENT_B = 0.6545;
    public static final double G_COEFFICENT_C = -1.741;
    public static final double G_COEFFICENT_D = 61.28;

  }
}
