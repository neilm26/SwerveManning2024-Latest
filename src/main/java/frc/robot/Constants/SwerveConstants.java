package frc.robot.Constants;


import java.util.HashMap;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveConstants {
    public static final int MODULE_COUNT = 4;
    public static final String CANIVORENAME = "canivore1";

    public static final double WHEEL_BASE = 0.5607; //cm to m, from one module adjacent to another.
    public static final double TRACK_WIDTH = 0.5607;

    public static final double WHEEL_DIAMETER = 3.00; // (inches)
    public static final double WHEEL_DIAMETER_METERS = 0.0762;
    public static final double TICKS_PER_REV_CIM_CODER = 1024;
    public static final double TICKS_PER_REV_ANALOG_CODER = 4096;


    public static final double HOME_ANALOG_ENC_POS_FRONT_RIGHT = -0.06944444444;
    public static final double HOME_ANALOG_ENC_POS_FRONT_LEFT = 0.1111111; //offsets necessary to zero out encoders
    public static final double HOME_ANALOG_ENC_POS_BACK_LEFT = -0.07777777777;
    public static final double HOME_ANALOG_ENC_POS_BACK_RIGHT = -0.0388888888888889;

    public static final double MAX_SPEED = 4.803648; //15.76 ft/sec to m/s
    public static final double MAX_CORRECTION_SPEED = 2.803648;
    public static final double MAX_ACCEL = 1.4;

    public static final double MAX_DRIVE_RPM = 5700;

    public static final double MAX_POINT_SPEED = 0.25;
    public static final double MAX_TURN_SPEED_SCALE = 1.0;

    public static final double TIMEOUT_MS = 5;
    public static final double LOOKAHEAD_S = 0.3;
    public static final double kEps = 1E-9;


    public static final double TEETH_PINION = 14;

    public static final double GEAR_RATIO = (45.0 * 22) / (TEETH_PINION * 15);

    public static final double DRIVE_VELOCITY_CONVERSION = (WHEEL_DIAMETER_METERS*Math.PI / GEAR_RATIO) / 60;
    public static final double DRIVE_POSITION_CONVERSION = (WHEEL_DIAMETER*Math.PI / GEAR_RATIO);

    //Driving encoder gear of module MK1 is 48 teeth.


    public static final Translation2d BACK_LEFT_OFFSET = new Translation2d(-WHEEL_BASE/2, -TRACK_WIDTH/2);
    public static final Translation2d BACK_RIGHT_OFFSET = new Translation2d(WHEEL_BASE/2, -TRACK_WIDTH/2);
    public static final Translation2d FRONT_RIGHT_OFFSET = new Translation2d(-WHEEL_BASE/2, TRACK_WIDTH/2);
    public static final Translation2d FRONT_LEFT_OFFSET = new Translation2d(WHEEL_BASE/2,TRACK_WIDTH/2);


    public static final Translation2d[] OFFSET_ARRAY = new Translation2d[] {FRONT_RIGHT_OFFSET, FRONT_LEFT_OFFSET, BACK_RIGHT_OFFSET, BACK_LEFT_OFFSET};


    public static double SPARK_PID_DRIVE_FF = 0.195;
    public static double SPARK_PID_TURN_FF = 0.11;


    public static PIDController DRIVE_PID_CONTROLLER = new PIDController(0.15, 0.000, 0.0);
    public static ProfiledPIDController ANGULAR_PID_CONTROLLER = new ProfiledPIDController(0.087, 0, 0, new TrapezoidProfile.Constraints(MAX_SPEED, MAX_ACCEL));

    public static PIDController COMPENSATION_PID_CONTROLLER = new PIDController(1, 0.04, 0.006);
    public static PIDController MODULE_HEADING_PID_CONTROLLER = new PIDController(0.0009, 0, 0);
    public static ProfiledPIDController MODULE_VELOCITY_PID_CONTROLLER = new ProfiledPIDController(0.4, 0, 0, new TrapezoidProfile.Constraints(MAX_SPEED, MAX_ACCEL));
    public static PIDController COMPENSATION_ANGULAR_PID_CONTROLLER = new PIDController(0.0011, 0.0002, 0.00009);

    
    public static SimpleMotorFeedforward TURN_FEEDFORWARD = new SimpleMotorFeedforward(0.06, 0.02);
    public static SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(1, 0.5);

    public static HolonomicDriveController SWERVE_PID_CONTROLLER = new HolonomicDriveController(DRIVE_PID_CONTROLLER, 
                                                                    DRIVE_PID_CONTROLLER, 
                                                                    ANGULAR_PID_CONTROLLER);    


    public static enum ModuleNames {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_RIGHT,
        BACK_LEFT
    }    

    public static HashMap<ModuleNames, Translation2d> offsetMap = new HashMap<ModuleNames, Translation2d>();

    public static void initializeOffsetMap() {
        offsetMap.put(ModuleNames.FRONT_RIGHT, OFFSET_ARRAY[0]);
        offsetMap.put(ModuleNames.FRONT_LEFT, OFFSET_ARRAY[1]);
        offsetMap.put(ModuleNames.BACK_RIGHT, OFFSET_ARRAY[2]);
        offsetMap.put(ModuleNames.BACK_LEFT, OFFSET_ARRAY[3]);
    }

    public static double[] DRIVE_PID_ARRAY = new double[] {0,0,0,0};
    public static double[] ANGULAR_PID_ARRAY = new double[] {0,0,0,0};

    public static final boolean TURNING_ENCODER_INVERTED = true;
    public static final double TURN_ENCODER_POS_FACTOR = (2*Math.PI);
    public static final double TURN_ENCODER_VELO_FACTOR = (2*Math.PI) / 60;
    public static final IdleMode TURN_MOTOR_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode DRIVE_MOTOR_IDLE_MODE = IdleMode.kCoast;

}