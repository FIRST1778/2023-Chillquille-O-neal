package org.chillout1778

import choreo.Choreo
import choreo.auto.AutoFactory
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.chillout1778.subsystems.*

object Robot : TimedRobot() {
    val isRedAlliance get() =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red

    val isOnRedSide get() =
        Swerve.estimatedPose.x > (Constants.Field.FIELD_X_SIZE / 2)

    val autoFactory = AutoFactory(
        { swervePose },
        { pose: Pose2d -> Swerve.resetPose(pose) },
        Swerve::followTrajectory,
        true,
        Swerve
    )

    var selectedAutoCommand: Command = InstantCommand()

    val autoChooser: SendableChooser<Command> = SendableChooser()

    var autoIsRunning = false

    override fun robotInit() {

        // Add autos to auto chooser in shuffleboard

    }

    object AutoContainer {
        // private val instant commands that you can mix and match
    }

    override fun teleopInit() {
    }


    override fun teleopExit() {
        Swerve.removeDefaultCommand()
        Superstructure.removeDefaultCommand()
    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
        Swerve.gyroAngle = 0.0
    }
}
