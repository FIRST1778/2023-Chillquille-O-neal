package org.chillout1778

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.chillout1778.subsystems.Arm
import org.chillout1778.subsystems.Elevator
import org.chillout1778.subsystems.Superstructure
import kotlin.math.abs

object Controls {
    private val driverController = CommandPS5Controller(0)
    private val operatorController = CommandPS5Controller(1)

    data class DriveInputs (

        var axial: Double = 0.0,
        var lateral: Double = 0.0,
        var rotate: Double = 0.0,
        var wantSwerveBrake: Boolean = false,
    )

    var autoControls = DriveInputs()

    val controls: DriveInputs get() {
        return if (Robot.isAutonomous) autoControls
        else DriveInputs(

            axial = -driverController.leftY,
            lateral = -driverController.leftX,
            rotate = -driverController.rightX,
            wantSwerveBrake = driverController.hid.l3Button,

            )
    }

    //For when we add autoalign
    fun autoAlignRumble(left: Double, right: Double) {
        driverController.setRumble(GenericHID.RumbleType.kLeftRumble, left)
        driverController.setRumble(GenericHID.RumbleType.kRightRumble, right)
    }
}
