package org.chillout1778

import choreo.Choreo
import choreo.auto.AutoFactory
import org.chillout1778.Elastic
import org.chillout1778.commands.TeleopDriveCommand
import org.chillout1778.subsystems.*
import org.chillout1778.subsystems.Swerve
import org.wpilib.command2.Command
import org.wpilib.command2.CommandScheduler
import org.wpilib.command2.Commands
import org.wpilib.command2.InstantCommand
import org.wpilib.framework.TimedRobot
import org.wpilib.smartdashboard.SendableChooser

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
        CommandScheduler.getInstance().run()
    }

    object AutoContainer {
        // private val instant commands that you can mix and match
    }

    // Select a specific tab on the Elastic Dashboard during Teleop
    override fun teleopInit() {
        Swerve.setDefaultCommand(TeleopDriveCommand(Controls::controls))
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
        Swerve.setDefaultCommand(Commands.none())
        Superstructure.setDefaultCommand(Commands.none())
    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }
}