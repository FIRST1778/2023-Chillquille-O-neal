package org.chillout1778.subsystems

import org.chillout1778.Constants
import org.chillout1778.Robot
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.estimation.TargetModel
import org.photonvision.targeting.PhotonPipelineResult
import org.wpilib.command2.SubsystemBase
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Pose3d
import org.wpilib.math.geometry.Rotation3d
import org.wpilib.math.geometry.Transform3d
import org.wpilib.math.geometry.Translation2d
import org.wpilib.math.linalg.VecBuilder
import org.wpilib.networktables.NetworkTableInstance
import org.wpilib.networktables.StructPublisher
import org.wpilib.util.sendable.SendableBuilder
import org.wpilib.vision.apriltag.AprilTagFieldLayout
import org.wpilib.vision.apriltag.AprilTagFields
import kotlin.jvm.optionals.getOrNull

object Vision : SubsystemBase() {


    class Camera(initialName: String, robotToCamera: Transform3d) : PhotonCamera(initialName) {
        val poseEstimator = PhotonPoseEstimator(AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.name), robotToCamera)
    }

    private val cameras = arrayOf(
        Camera(Constants.Vision.CAMERA1_NAME, Constants.Vision.CAMERA1_TRANSFORM)
    )

    fun removeResult(res: PhotonPipelineResult): Boolean {
        return false
    }

    fun periodicAddMeasurements() {
        for (camera in cameras) {
            val results = camera.getAllUnreadResults().filterNot {
                removeResult(it)
            }

            for (result in results) {
                // Try coprocessor multi-tag first
                val poseResult =
                    camera.poseEstimator.estimateCoprocMultiTagPose(result)
                        .orElseGet {
                            camera.poseEstimator.estimateLowestAmbiguityPose(result).orElse(null)
                        }

                if (poseResult != null) {
                    Swerve.addVisionMeasurement(
                        poseResult.estimatedPose.toPose2d(),
                        poseResult.timestampSeconds
                    )
                }
            }
        }
    }

    var turretPose = Pose3d(0.0, 0.0, 0.0, Rotation3d(0.0, 0.0, 0.0))

    val estimatedPosePublisher : StructPublisher<Pose2d> = NetworkTableInstance.getDefault().getStructTopic("/Field/RobotPose", Pose2d.struct).publish();

    val cameraPosePublisher : StructPublisher<Pose3d> = NetworkTableInstance.getDefault().getStructTopic("/Field/CameraPose", Pose3d.struct).publish();

    override fun periodic() {
        periodicAddMeasurements()
        estimatedPosePublisher.set(Swerve.pose)
        cameraPosePublisher.set(turretPose)
    }

    override fun initSendable(builder: SendableBuilder) {

    }
}