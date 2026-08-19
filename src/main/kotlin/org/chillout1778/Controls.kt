package org.chillout1778

import org.wpilib.driverstation.NiDsPS5Controller
import org.wpilib.driverstation.GenericHID


object Controls {
    private val driverController = NiDsPS5Controller(0)
    private val operatorController = NiDsPS5Controller(1)

    data class DriveInputs (

        var axial: Double = 0.0,
        var lateral: Double = 0.0,
        var rotate: Double = 0.0,
        var wantSwerveBrake: Boolean = false,
    )

    var autoControls = DriveInputs()

    val controls: DriveInputs get() {
        return if (Robot.autoIsRunning) autoControls
        else DriveInputs(

            axial = -driverController.leftY,
            lateral = -driverController.leftX,
            rotate = -driverController.rightX,
            wantSwerveBrake = driverController.l3Button,

            )
    }

    //For when we add autoalign
    fun autoAlignRumble(left: Double, right: Double) {
        driverController.setRumble(GenericHID.RumbleType.LEFT_RUMBLE, left)
        driverController.setRumble(GenericHID.RumbleType.RIGHT_RUMBLE, right)
    }
}
