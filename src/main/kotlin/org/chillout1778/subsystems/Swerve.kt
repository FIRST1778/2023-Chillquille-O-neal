package org.chillout1778.subsystems

import choreo.trajectory.SwerveSample
import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Twist2d
import edu.wpi.first.math.kinematics.proto.ChassisSpeedsProto
import edu.wpi.first.math.kinematics.struct.ChassisSpeedsStruct
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.util.protobuf.ProtobufSerializable
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.sendable.SendableRegistry
import edu.wpi.first.util.struct.StructSerializable
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import org.chillout1778.Constants
import org.chillout1778.generated.TunerConstants.*
import java.util.*
import java.util.function.Supplier

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
object Swerve : TunerSwerveDrivetrain(
    DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight), Subsystem, Sendable {
    private var m_simNotifier: Notifier? = null
    private var m_lastSimTime = 0.0

    /* Keep track if we've ever applied the operator perspective before or not */
    private var m_hasAppliedOperatorPerspective = false

    /* Swerve requests to apply during SysId characterization */
    private val m_translationCharacterization = SwerveRequest.SysIdSwerveTranslation()
    private val m_steerCharacterization = SwerveRequest.SysIdSwerveSteerGains()
    private val m_rotationCharacterization = SwerveRequest.SysIdSwerveRotation()

    private const val kSimLoopPeriod = 0.005 // 5 ms

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private val kBlueAlliancePerspectiveRotation: Rotation2d = Rotation2d.kZero

    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private val kRedAlliancePerspectiveRotation: Rotation2d = Rotation2d.k180deg

    init {
        SendableRegistry.add(this, name, name)

        if (Utils.isSimulation()) {
            startSimThread()
        }

        modules.forEach { module ->
            module.driveMotor.optimizeBusUtilization()
            module.steerMotor.optimizeBusUtilization()
            module.encoder.optimizeBusUtilization()
        }
    }

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private val m_sysIdRoutineTranslation = SysIdRoutine(
        SysIdRoutine.Config(
            null,  // Use default ramp rate (1 V/s)
            Units.Volts.of(4.0),  // Reduce dynamic step voltage to 4 V to prevent brownout
            null
        )  // Use default timeout (10 s)
        // Log state with SignalLogger class
        { state: SysIdRoutineLog.State -> SignalLogger.writeString("SysIdTranslation_State", state.toString()) },
        Mechanism(
            { output: Voltage? -> setControl(m_translationCharacterization.withVolts(output)) },
            null,
            this
        )
    )

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private val m_sysIdRoutineSteer = SysIdRoutine(
        SysIdRoutine.Config(
            null,  // Use default ramp rate (1 V/s)
            Units.Volts.of(7.0),  // Use dynamic voltage of 7 V
            null
        )  // Use default timeout (10 s)
        // Log state with SignalLogger class
        { state: SysIdRoutineLog.State -> SignalLogger.writeString("SysIdSteer_State", state.toString()) },
        Mechanism(
            { volts: Voltage? -> setControl(m_steerCharacterization.withVolts(volts)) },
            null,
            this
        )
    )

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private val m_sysIdRoutineRotation = SysIdRoutine(
        SysIdRoutine.Config( /* This is in radians per second², but SysId only supports "volts per second" */
            Units.Volts.of(Math.PI / 6)
                .per(Units.Second),  /* This is in radians per second, but SysId only supports "volts" */
            Units.Volts.of(Math.PI),
            null
        )  // Use default timeout (10 s)
        // Log state with SignalLogger class
        { state: SysIdRoutineLog.State -> SignalLogger.writeString("SysIdRotation_State", state.toString()) },
        Mechanism(
            { output: Voltage ->
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.`in`(Units.Volts)))
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.`in`(Units.Volts))
            },
            null,
            this
        )
    )

    /* The SysId routine to test */
    private val m_sysIdRoutineToApply = m_sysIdRoutineTranslation

    //region old constructors removed to stop compile errors
//    /**
//     * Constructs a CTRE SwerveDrivetrain using the specified constants.
//     *
//     *
//     * This constructs the underlying hardware devices, so users should not construct
//     * the devices themselves. If they need the devices, they can access them through
//     * getters in the classes.
//     *
//     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
//     * @param modules               Constants for each specific module
//     */
//    constructor(
//        drivetrainConstants: SwerveDrivetrainConstants,
//        vararg modules: SwerveModuleConstants<*, *, *>?
//    ) : super(drivetrainConstants, *modules) {
//        if (Utils.isSimulation()) {
//            startSimThread()
//        }
//    }
//
//    /**
//     * Constructs a CTRE SwerveDrivetrain using the specified constants.
//     *
//     *
//     * This constructs the underlying hardware devices, so users should not construct
//     * the devices themselves. If they need the devices, they can access them through
//     * getters in the classes.
//     *
//     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
//     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
//     * unspecified or set to 0 Hz, this is 250 Hz on
//     * CAN FD, and 100 Hz on CAN 2.0.
//     * @param modules                 Constants for each specific module
//     */
//    constructor(
//        drivetrainConstants: SwerveDrivetrainConstants,
//        odometryUpdateFrequency: Double,
//        vararg modules: SwerveModuleConstants<*, *, *>?
//    ) : super(drivetrainConstants, odometryUpdateFrequency, *modules) {
//        if (Utils.isSimulation()) {
//            startSimThread()
//        }
//    }
//
//    /**
//     * Constructs a CTRE SwerveDrivetrain using the specified constants.
//     *
//     *
//     * This constructs the underlying hardware devices, so users should not construct
//     * the devices themselves. If they need the devices, they can access them through
//     * getters in the classes.
//     *
//     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
//     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
//     * unspecified or set to 0 Hz, this is 250 Hz on
//     * CAN FD, and 100 Hz on CAN 2.0.
//     * @param odometryStandardDeviation The standard deviation for odometry calculation
//     * in the form [x, y, theta]ᵀ, with units in meters
//     * and radians
//     * @param visionStandardDeviation   The standard deviation for vision calculation
//     * in the form [x, y, theta]ᵀ, with units in meters
//     * and radians
//     * @param modules                   Constants for each specific module
//     */
//    constructor(
//        drivetrainConstants: SwerveDrivetrainConstants,
//        odometryUpdateFrequency: Double,
//        odometryStandardDeviation: Matrix<N3?, N1?>,
//        visionStandardDeviation: Matrix<N3?, N1?>,
//        vararg modules: SwerveModuleConstants<*, *, *>?
//    ) : super(
//        drivetrainConstants,
//        odometryUpdateFrequency,
//        odometryStandardDeviation,
//        visionStandardDeviation,
//        *modules
//    ) {
//        if (Utils.isSimulation()) {
//            startSimThread()
//        }
//    }
    //endregion

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    @Deprecated(message = "Don't use this one!", replaceWith = ReplaceWith("TeleOpDriveCommand"))
    fun applyRequest(requestSupplier: Supplier<SwerveRequest?>): Command {
        return run { this.setControl(requestSupplier.get()) }
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by [.m_sysIdRoutineToApply].
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    fun sysIdQuasistatic(direction: SysIdRoutine.Direction?): Command {
        return m_sysIdRoutineToApply.quasistatic(direction)
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by [.m_sysIdRoutineToApply].
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    fun sysIdDynamic(direction: SysIdRoutine.Direction?): Command {
        return m_sysIdRoutineToApply.dynamic(direction)
    }

    val xController = PIDController(Constants.Swerve.DRIVE_KP, Constants.Swerve.DRIVE_KI, Constants.Swerve.DRIVE_KD)
    val yController = PIDController(Constants.Swerve.DRIVE_KP, Constants.Swerve.DRIVE_KI, Constants.Swerve.DRIVE_KD)
    val headingController = PIDController(Constants.Swerve.HEADING_KP, Constants.Swerve.HEADING_KI, Constants.Swerve.HEADING_KD).apply{
        enableContinuousInput(-Math.PI, Math.PI)
    }

    private val swerveRequest = SwerveRequest.ApplyFieldSpeeds()

    class ChassisSpeeds : ProtobufSerializable, StructSerializable {
        var vxMetersPerSecond: Double = 0.0
        var vyMetersPerSecond: Double = 0.0
        var omegaRadiansPerSecond: Double = 0.0

        constructor()

        constructor(vxMetersPerSecond: Double, vyMetersPerSecond: Double, omegaRadiansPerSecond: Double) {
            this.vxMetersPerSecond = vxMetersPerSecond
            this.vyMetersPerSecond = vyMetersPerSecond
            this.omegaRadiansPerSecond = omegaRadiansPerSecond
        }

        constructor(vx: LinearVelocity, vy: LinearVelocity, omega: AngularVelocity) : this(
            vx.`in`(Units.MetersPerSecond), vy.`in`(
                Units.MetersPerSecond
            ), omega.`in`(Units.RadiansPerSecond)
        )

        fun toTwist2d(dtSeconds: Double): Twist2d {
            return Twist2d(
                this.vxMetersPerSecond * dtSeconds,
                this.vyMetersPerSecond * dtSeconds,
                this.omegaRadiansPerSecond * dtSeconds
            )
        }

        fun plus(other: ChassisSpeeds): ChassisSpeeds {
            return ChassisSpeeds(
                this.vxMetersPerSecond + other.vxMetersPerSecond,
                this.vyMetersPerSecond + other.vyMetersPerSecond,
                this.omegaRadiansPerSecond + other.omegaRadiansPerSecond
            )
        }

        fun minus(other: ChassisSpeeds): ChassisSpeeds {
            return ChassisSpeeds(
                this.vxMetersPerSecond - other.vxMetersPerSecond,
                this.vyMetersPerSecond - other.vyMetersPerSecond,
                this.omegaRadiansPerSecond - other.omegaRadiansPerSecond
            )
        }

        fun unaryMinus(): ChassisSpeeds {
            return ChassisSpeeds(-this.vxMetersPerSecond, -this.vyMetersPerSecond, -this.omegaRadiansPerSecond)
        }

        fun times(scalar: Double): ChassisSpeeds {
            return ChassisSpeeds(
                this.vxMetersPerSecond * scalar,
                this.vyMetersPerSecond * scalar,
                this.omegaRadiansPerSecond * scalar
            )
        }

        fun div(scalar: Double): ChassisSpeeds {
            return ChassisSpeeds(
                this.vxMetersPerSecond / scalar,
                this.vyMetersPerSecond / scalar,
                this.omegaRadiansPerSecond / scalar
            )
        }

        override fun hashCode(): Int {
            return Objects.hash(
                *arrayOf<Any>(
                    this.vxMetersPerSecond,
                    this.vyMetersPerSecond,
                    this.omegaRadiansPerSecond
                )
            )
        }

        override fun toString(): String {
            return String.format(
                "ChassisSpeeds(Vx: %.2f m/s, Vy: %.2f m/s, Omega: %.2f rad/s)",
                this.vxMetersPerSecond,
                this.vyMetersPerSecond,
                this.omegaRadiansPerSecond
            )
        }

        companion object {
            val proto: ChassisSpeedsProto = ChassisSpeedsProto()
            val struct: ChassisSpeedsStruct = ChassisSpeedsStruct()

            fun discretize(
                vxMetersPerSecond: Double,
                vyMetersPerSecond: Double,
                omegaRadiansPerSecond: Double,
                dtSeconds: Double
            ): ChassisSpeeds {
                val desiredDeltaPose = Pose2d(
                    vxMetersPerSecond * dtSeconds,
                    vyMetersPerSecond * dtSeconds,
                    Rotation2d(omegaRadiansPerSecond * dtSeconds)
                )
                val twist = Pose2d.kZero.log(desiredDeltaPose)
                return ChassisSpeeds(twist.dx / dtSeconds, twist.dy / dtSeconds, twist.dtheta / dtSeconds)
            }

            fun discretize(vx: LinearVelocity, vy: LinearVelocity, omega: AngularVelocity, dt: Time): ChassisSpeeds {
                return discretize(
                    vx.`in`(Units.MetersPerSecond),
                    vy.`in`(Units.MetersPerSecond),
                    omega.`in`(Units.RadiansPerSecond),
                    dt.`in`(
                        Units.Seconds
                    )
                )
            }

            fun discretize(continuousSpeeds: ChassisSpeeds, dtSeconds: Double): ChassisSpeeds {
                return discretize(
                    continuousSpeeds.vxMetersPerSecond,
                    continuousSpeeds.vyMetersPerSecond,
                    continuousSpeeds.omegaRadiansPerSecond,
                    dtSeconds
                )
            }

            fun fromFieldRelativeSpeeds(
                vxMetersPerSecond: Double,
                vyMetersPerSecond: Double,
                omegaRadiansPerSecond: Double,
                robotAngle: Rotation2d
            ): ChassisSpeeds {
                val rotated = (Translation2d(vxMetersPerSecond, vyMetersPerSecond)).rotateBy(robotAngle.unaryMinus())
                return ChassisSpeeds(rotated.x, rotated.y, omegaRadiansPerSecond)
            }

            fun fromFieldRelativeSpeeds(
                vx: LinearVelocity,
                vy: LinearVelocity,
                omega: AngularVelocity,
                robotAngle: Rotation2d
            ): ChassisSpeeds {
                return fromFieldRelativeSpeeds(
                    vx.`in`(Units.MetersPerSecond), vy.`in`(Units.MetersPerSecond), omega.`in`(
                        Units.RadiansPerSecond
                    ), robotAngle
                )
            }

            fun fromFieldRelativeSpeeds(fieldRelativeSpeeds: ChassisSpeeds, robotAngle: Rotation2d): ChassisSpeeds {
                return fromFieldRelativeSpeeds(
                    fieldRelativeSpeeds.vxMetersPerSecond,
                    fieldRelativeSpeeds.vyMetersPerSecond,
                    fieldRelativeSpeeds.omegaRadiansPerSecond,
                    robotAngle
                )
            }

            fun fromRobotRelativeSpeeds(
                vxMetersPerSecond: Double,
                vyMetersPerSecond: Double,
                omegaRadiansPerSecond: Double,
                robotAngle: Rotation2d?
            ): ChassisSpeeds {
                val rotated = (Translation2d(vxMetersPerSecond, vyMetersPerSecond)).rotateBy(robotAngle)
                return ChassisSpeeds(rotated.x, rotated.y, omegaRadiansPerSecond)
            }

            fun fromRobotRelativeSpeeds(
                vx: LinearVelocity,
                vy: LinearVelocity,
                omega: AngularVelocity,
                robotAngle: Rotation2d?
            ): ChassisSpeeds {
                return fromRobotRelativeSpeeds(
                    vx.`in`(Units.MetersPerSecond), vy.`in`(Units.MetersPerSecond), omega.`in`(
                        Units.RadiansPerSecond
                    ), robotAngle
                )
            }

            fun fromRobotRelativeSpeeds(robotRelativeSpeeds: ChassisSpeeds, robotAngle: Rotation2d?): ChassisSpeeds {
                return fromRobotRelativeSpeeds(
                    robotRelativeSpeeds.vxMetersPerSecond,
                    robotRelativeSpeeds.vyMetersPerSecond,
                    robotRelativeSpeeds.omegaRadiansPerSecond,
                    robotAngle
                )
            }
        }
    }
    override fun periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent { allianceColor: Alliance ->
                setOperatorPerspectiveForward(
                    if (allianceColor == Alliance.Red)
                        kRedAlliancePerspectiveRotation
                    else
                        kBlueAlliancePerspectiveRotation
                )
                m_hasAppliedOperatorPerspective = true
            }
        }
    }

    private fun startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds()

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = Notifier {
            val currentTime = Utils.getCurrentTimeSeconds()
            val deltaTime = currentTime - m_lastSimTime
            m_lastSimTime = currentTime

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage())
        }
        m_simNotifier!!.startPeriodic(kSimLoopPeriod)
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    override fun addVisionMeasurement(visionRobotPoseMeters: Pose2d, timestampSeconds: Double) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds))
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     *
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * [.setVisionMeasurementStdDevs] or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     * in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    override fun addVisionMeasurement(
        visionRobotPoseMeters: Pose2d,
        timestampSeconds: Double,
        visionMeasurementStdDevs: Matrix<N3, N1>
    ) {
        super.addVisionMeasurement(
            visionRobotPoseMeters,
            Utils.fpgaToCurrentTime(timestampSeconds),
            visionMeasurementStdDevs
        )
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.addDoubleProperty("Voltage", { modules[0].driveMotor.motorVoltage.valueAsDouble }, {})
        builder.addDoubleProperty("Velocity", { state.ModuleStates[0].speed }, {})

        builder.addDoubleProperty("Drive Motor Supply Current", {modules[0].driveMotor.supplyCurrent.valueAsDouble }, {})
        builder.addDoubleProperty("Steer Motor Supply Current", {modules[0].steerMotor.supplyCurrent.valueAsDouble }, {})

        builder.addDoubleProperty("Drive Motor Supply Voltage", {modules[0].driveMotor.supplyVoltage.valueAsDouble }, {})
        builder.addDoubleProperty("Steer Motor Supply Voltage", {modules[0].steerMotor.supplyVoltage.valueAsDouble }, {})
    }

}
