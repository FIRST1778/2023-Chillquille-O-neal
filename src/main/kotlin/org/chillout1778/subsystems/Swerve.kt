package org.chillout1778.subsystems

import choreo.trajectory.SwerveSample
import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.signals.InvertedValue
import org.chillout1778.Robot
import org.chillout1778.Constants
import org.chillout1778.subsystems.Elevator.height
import org.wpilib.command2.SubsystemBase
import org.wpilib.math.controller.PIDController
import org.wpilib.math.estimator.SwerveDrivePoseEstimator
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Translation2d
import org.wpilib.math.kinematics.SwerveDriveKinematics
import org.wpilib.math.linalg.VecBuilder
import org.wpilib.math.util.MathUtil
import org.wpilib.smartdashboard.Field2d
import org.wpilib.smartdashboard.SmartDashboard
import kotlin.math.abs

object Swerve: SubsystemBase() {
    val ENCODER_CAN_BUS = CANBus.systemCore(2)
    val DRIVE_CAN_BUS = CANBus.systemCore(0)
    val TURN_CAN_BUS = CANBus.systemCore(1)

    private val gyro = Pigeon2(Constants.CanIds.GYRO).apply {
        configurator.apply(
            Pigeon2Configuration()
        )
    }

    // Read yaw from the gyro (radians, counterclockwise from forward).
    var gyroAngle: Double
        get() = MathUtil.angleModulus(gyro.rotation2d.radians)
        set(desiredAngle) {
            gyro.setYaw(MathUtil.angleModulus(desiredAngle) / Math.PI * 180)
        }

    // The order doesn't matter to us, but PathPlannerLib prefers FL, FR, BL, BR.
    private val modules = arrayOf(
        SwerveModule(
            name = "Front Left",
            driveMotorID = Constants.CanIds.SWERVE_FRONT_LEFT_DRIVE,
            turnMotorID = Constants.CanIds.SWERVE_FRONT_LEFT_TURN,
            canCoderID = Constants.CanIds.SWERVE_FRONT_LEFT_CANCODER,
            driveInverted = InvertedValue.CounterClockwise_Positive,
            turnInverted = InvertedValue.Clockwise_Positive,
            encoderOffset = Constants.Swerve.FRONT_LEFT_ENCODER_OFFSET,
            encoderCanBus = ENCODER_CAN_BUS,
            driveCanBus = DRIVE_CAN_BUS,
            turnCanBus = TURN_CAN_BUS
        ),
        SwerveModule(
            name = "Front Right",
            driveMotorID = Constants.CanIds.SWERVE_FRONT_RIGHT_DRIVE,
            turnMotorID = Constants.CanIds.SWERVE_FRONT_RIGHT_TURN,
            canCoderID = Constants.CanIds.SWERVE_FRONT_RIGHT_CANCODER,
            driveInverted = InvertedValue.Clockwise_Positive,
            turnInverted = InvertedValue.Clockwise_Positive,
            encoderOffset = Constants.Swerve.FRONT_RIGHT_ENCODER_OFFSET,
            encoderCanBus = ENCODER_CAN_BUS,
            driveCanBus = DRIVE_CAN_BUS,
            turnCanBus = TURN_CAN_BUS
        ),
        SwerveModule(
            name = "Back Left",
            driveMotorID = Constants.CanIds.SWERVE_BACK_LEFT_DRIVE,
            turnMotorID = Constants.CanIds.SWERVE_BACK_LEFT_TURN,
            canCoderID = Constants.CanIds.SWERVE_BACK_LEFT_CANCODER,
            driveInverted = InvertedValue.CounterClockwise_Positive,
            turnInverted = InvertedValue.Clockwise_Positive,
            encoderOffset = Constants.Swerve.BACK_LEFT_ENCODER_OFFSET,
            encoderCanBus = ENCODER_CAN_BUS,
            driveCanBus = DRIVE_CAN_BUS,
            turnCanBus = TURN_CAN_BUS
        ),
        SwerveModule(
            name = "Back Right",
            driveMotorID = Constants.CanIds.SWERVE_BACK_RIGHT_DRIVE,
            turnMotorID = Constants.CanIds.SWERVE_BACK_RIGHT_TURN,
            canCoderID = Constants.CanIds.SWERVE_BACK_RIGHT_CANCODER,
            driveInverted = InvertedValue.Clockwise_Positive,
            turnInverted = InvertedValue.Clockwise_Positive,
            encoderOffset = Constants.Swerve.BACK_RIGHT_ENCODER_OFFSET,
            encoderCanBus = ENCODER_CAN_BUS,
            driveCanBus = DRIVE_CAN_BUS,
            turnCanBus = TURN_CAN_BUS
        )
    )

    private val fieldEstimate = Field2d()
//    val goalField = Field2d().apply { name = "goalPose" }

    var isAligned = false

    init {
        for (module in modules) {
            SmartDashboard.putData(module.name, module)
        }
        SmartDashboard.putData("swerve actual object", this)
        SmartDashboard.putData("Swerve Estimated Pose Field", fieldEstimate)
//        Shuffleboard.getTab("Swerve").add("Goal Pose", goalField)
    }

    private val kinematics = SwerveDriveKinematics(
        Translation2d(Constants.Swerve.XY_DISTANCE, Constants.Swerve.XY_DISTANCE), // FL
        Translation2d(Constants.Swerve.XY_DISTANCE, -Constants.Swerve.XY_DISTANCE), // FR
        Translation2d(-Constants.Swerve.XY_DISTANCE, Constants.Swerve.XY_DISTANCE), // BL
        Translation2d(-Constants.Swerve.XY_DISTANCE, -Constants.Swerve.XY_DISTANCE), // BR
    )

    private val poseEstimator = SwerveDrivePoseEstimator(
        kinematics, // positions of modules
        Rotation2d(gyroAngle), // initial robot yaw (converted to Rotation2d)
        modulePositions, // initial "positions" (how far the wheels have moved and in what direction)
        Pose2d(0.0, 0.0, Rotation2d(gyroAngle)),
        VecBuilder.fill(0.1, 0.1, 0.1), // odometry
        VecBuilder.fill(.9, .9, 2.0) // vision
    )

    override fun periodic() {
        Vision.periodicAddMeasurements(poseEstimator)
        poseEstimator.update(
            Rotation2d(gyroAngle),
            modulePositions
        )
        fieldEstimate.robotPose = poseEstimator.estimatedPosition
    }

    fun driveRobotRelative(speeds: ChassisSpeeds) {
        val discreteSpeeds = speeds.discretize(Robot.period)
        // val discreteSpeeds = ChassisSpeeds.discretize(speeds, Robot.period)
        val moduleStates = kinematics.toSwerveModuleStates(discreteSpeeds)
        for ((mod, state) in modules.zip(moduleStates)) {
            mod.driveState(state)
        }
        poseEstimator.update(Rotation2d(gyroAngle), modulePositions)
    }

    fun stop() {
        driveRobotRelative(ChassisSpeeds())
    }

    private val xController = PIDController(12.0, 0.0, 0.0)
    private val yController = PIDController(12.0, 0.0, 0.0)
    private val headingController = PIDController(5.0, 0.0, 0.0).apply {
        enableContinuousInput(-Math.PI, Math.PI)
    }

    fun followSample(sample: SwerveSample) {
        val pose = poseEstimator.estimatedPosition
//        goalField.robotPose = sample.pose

        val speeds = ChassisSpeeds(
            sample.vx + xController.calculate(pose.x, sample.x),
            sample.vy + yController.calculate(pose.y, sample.y),
            sample.omega + headingController.calculate(pose.rotation.radians, sample.heading)
        )
    }

    fun followPose(goalPose: Pose2d) {
        val currentPose = poseEstimator.estimatedPosition
        val speeds = ChassisSpeeds(
            xController.calculate(currentPose.x, goalPose.x),
            yController.calculate(currentPose.y, goalPose.y),
            headingController.calculate(currentPose.rotation.radians, goalPose.rotation.radians)
        )
    }
}

