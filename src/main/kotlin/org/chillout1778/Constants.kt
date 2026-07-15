package org.chillout1778

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.util.Units
import org.chillout1778.Utils.mirror
import kotlin.math.PI
import kotlin.math.sqrt


object Constants{
    object CanIds{
        //Don't put anything on ID 0, when a new motor is recognized in Phoenix Tuner X it causes issues
        // SWERVE MOTORS !
        const val SWERVE_FRONT_LEFT_DRIVE = 1
        const val SWERVE_FRONT_LEFT_TURN = 2
        const val SWERVE_FRONT_RIGHT_DRIVE = 3
        const val SWERVE_FRONT_RIGHT_TURN = 4
        const val SWERVE_BACK_RIGHT_DRIVE = 5
        const val SWERVE_BACK_RIGHT_TURN = 6
        const val SWERVE_BACK_LEFT_DRIVE = 7
        const val SWERVE_BACK_LEFT_TURN = 8
        const val SWERVE_FRONT_LEFT_CANCODER = 9
        const val SWERVE_FRONT_RIGHT_CANCODER = 10
        const val SWERVE_BACK_LEFT_CANCODER = 11
        const val SWERVE_BACK_RIGHT_CANCODER = 12

        // TILTEDEVATOR MOTORS !

        const val ELEVATOR_MAIN_MOTOR = 13
        const val ELEVATOR_FOLLOWER_MOTOR = 14

        // ARM MOTORS !

        const val ARM_WRIST_MOTOR = 15

        const val ARM_SHOULDER_MAIN_MOTOR = 16
        const val ARM_SHOULDER_FOLLOWER_MOTOR = 17

        // MANIPULATOR MOTORS !

        const val MANIPULATOR_ROLLER_MAIN_MOTOR = 18
        const val MANIPULATOR_ROLLER_FOLLOER_MOTOR = 19

        // GYRO

    }
    object DioIds{
        // Encoders, sensors (cube/cone detection later on)

        const val MANIPULATOR_LINE_BREAK = 0
        const val BRAKE_MODE_SWITCH = 1

    }
    object Vision{
        val FIELD_TYPE = AprilTagFields.k2023ChargedUp

        // Insert cameras and positions here
    }
    object Field {
        const val FIELD_X_SIZE = 16.541750
        const val FIELD_Y_SIZE = 8.013700

        // Scoring poses here !
    }

    // InterpolatingDoubleTreeMap because robot uses a tilted linear axis with a rotating pivot...LUT is too jerky
   // var pivotElevatorPairs = listOf(
        //pivot location to elevator location (in rads)

    //)

    // KEEPING SWERVE CODE FROM SUBZ
    object Swerve{
        val ALIGNMENT_TOLERANCE: Double = 0.04
        val STARTING_TOLERANCE: Double = 0.15

        // TODO: we should do motion magic here too
        // Module constants
        val WHEEL_RADIUS = Units.inchesToMeters(3.9/2)
        const val DRIVE_RATIO = 1.0 / ((16.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0))
        const val TURN_RATIO =  150.0 / 7.0

        // These turn PID values are borrowed straight from Zappy.
        fun makeTurnPID() = PIDController(7.0, 0.0, 0.01).apply {
            enableContinuousInput(-Math.PI, Math.PI)
        }
        fun makeDriveFeedforward() = SimpleMotorFeedforward(
            0.2199442,
            2.18943902193,
            0.0
        )
        fun makeAlignTurnPID() = PIDController(6.0, 0.0, 0.04).apply {
            enableContinuousInput(-Math.PI, Math.PI)
        }
        fun makeAlignDrivePID() = PIDController(5.0, 0.0, 0.01)
        fun makeBargeAlignDrivePID() = PIDController(6.0, 0.0, 0.0)
        val maxAlignTranslationSpeed = 1.5
        val maxAlignRotationSpeed = 2.5
        val maxBargeAlignTranslationSpeed = 1.5
        val maxBargeAlignRotationSpeed = 1.5
        const val MAX_NODE_DISTANCE = 3.0 // meters
        // How fast the robot can move in a straight line (meters/sec).
        val MAX_VELOCITY = (5800.0 / 60) / DRIVE_RATIO * WHEEL_RADIUS * 2*PI
        // How far the swerve modules are from (0,0).
        val XY_DISTANCE = Units.inchesToMeters(13.393747)
        // How fast the robot can rotate (radians/sec).
        val MAX_ANGULAR_VELOCITY = MAX_VELOCITY / (XY_DISTANCE * sqrt(2.0))
        val CHASSIS_RADIUS = (XY_DISTANCE * sqrt(2.0))

        val ALIGN_ANGLE_WEIGHT = 2.7
        val ALIGN_TRANSLATION_WEIGHT = 5.0
        val ALREADY_SCORED_BADNESS = 0.5 + Units.inchesToMeters(12.9375) * ALIGN_TRANSLATION_WEIGHT * (1 - 2*0.3)

        const val FRONT_LEFT_ENCODER_OFFSET = 0.44 // ROTATIONS
        const val FRONT_RIGHT_ENCODER_OFFSET = 0.276 // ROTATIONS
        const val BACK_RIGHT_ENCODER_OFFSET = 0.372 // ROTATIONS
        const val BACK_LEFT_ENCODER_OFFSET = 0.298 // ROTATIONS
    }

    object Pivot{
        val PIVOT_CONFIG = TalonFXConfiguration().apply {
            // Motor output
            // Motion magic (?) + current limits
        }

        // operating range + encoder ratios

        // Safeties

        // setpoint

        // current draw
    }
    object Elevator{
        // Max extension, setpoints, current draw

        val MOTOR_CONFIG = TalonFXConfiguration().apply {
            // slots, gear ratios, spools, motion magic
        }
    }