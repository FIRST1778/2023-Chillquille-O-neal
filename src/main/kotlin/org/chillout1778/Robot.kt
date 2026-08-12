package org.chillout1778
import org.chillout1778.Elastic

import choreo.Choreo
import choreo.auto.AutoFactory
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.chillout1778.commands.TeleopDriveCommand
import org.chillout1778.org.chillout1778.subsystems.ArmElevatorManager
import org.chillout1778.subsystems.*
import org.chillout1778.subsystems.Swerve

object Robot : TimedRobot() {

    var selectedAutoCommand: Command = InstantCommand()

    val autoChooser: SendableChooser<Command> = SendableChooser()

    init {
        // MUST BE CALLED FIRST before subsystems boot up!
        LoggingManager.start()
    }

    var autoIsRunning = false

    override fun robotInit() {
        // Add autos to auto chooser in shuffleboard

    }

    override fun robotPeriodic() {
        ArmElevatorManager.update()
        CommandScheduler.getInstance().run()

    }

    object AutoContainer {
        // private val instant commands that you can mix and match
    }


    // Select a specific tab on the Elastic Dashboard during Teleop
    override fun teleopInit() {
        Swerve.defaultCommand = TeleopDriveCommand(Controls::controls)
        autoIsRunning = false
        Elastic.selectTab("Teleop")

        Elastic.sendNotification(
            Elastic.Notification.builder()
                .withTitle("Teleop Active")
                .withMessage("Driver controls initialized.")
                .withLevel(Elastic.Notification.Level.INFO)
                .build()
        )
    }


    override fun teleopExit() {
        Swerve.removeDefaultCommand()
        Superstructure.removeDefaultCommand()
    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }
}
