//package org.chillout1778.subsystems
//
//import edu.wpi.first.apriltag.AprilTagFieldLayout
//import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
//import edu.wpi.first.math.geometry.Transform3d
//import edu.wpi.first.util.sendable.SendableBuilder
//import edu.wpi.first.wpilibj2.command.SubsystemBase
//import org.chillout1778.Constants
//import org.photonvision.PhotonCamera
//import org.photonvision.PhotonPoseEstimator
//import org.photonvision.estimation.TargetModel
//import org.photonvision.targeting.PhotonPipelineResult
//
//
//import kotlin.jvm.optionals.getOrNull
//import kotlin.math.abs
//import kotlin.math.atan2
//import kotlin.math.sqrt
//
//object Vision: SubsystemBase(){
//    class Camera(initialName: String, robotToCamera: Transform3d) : PhotonCamera(initialName) {
//        val poseEstimator = PhotonPoseEstimator(
//            AprilTagFieldLayout.loadField(Constants.Vision.FIELD_TYPE),
//            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//            robotToCamera
//        ).apply {
//            tagModel = TargetModel.kAprilTag36h11
//            setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY)
//        }
//    }
//
//    private val cameras = arrayOf(
//
//    )
//
//    fun allConnected() = cameras.all { it.isConnected() }
//
//    // What does this mean?
//    fun removeResult(res: PhotonPipelineResult): Boolean {
//        return res.getTargets().map { it.fiducialId }.any { it in listOf(4, 5, 14, 15) }
//    }
//
//    fun periodicAddMeasurements(estimator: SwerveDrivePoseEstimator) {
//
//    }
//
//
//    override fun initSendable(builder: SendableBuilder) {
//        for(camera in cameras){
//            builder.addBooleanProperty(camera.name + " connection status", {camera.isConnected}, {})
//        }
//    }
//}