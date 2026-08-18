package org.chillout1778.commands

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import java.util.function.Supplier
import org.chillout1778.Constants
import org.chillout1778.Controls.DriveInputs
import org.chillout1778.subsystems.Swerve
import org.wpilib.command2.Command
import org.wpilib.math.kinematics.ChassisVelocities

class TeleopDriveCommand(
    private val driveInputsSupplier: Supplier<DriveInputs>
) : Command() {

    init {
        addRequirements(Swerve)
    }


    override fun execute() {
        val inputs = driveInputsSupplier.get()

        if (inputs.wantSwerveBrake) {
            Swerve.stop()
        } else {
            Swerve.driveRobotRelative(
                ChassisVelocities(
                    inputs.axial * Constants.Swerve.MAX_SPEED,
                    inputs.lateral * Constants.Swerve.MAX_SPEED,
                    inputs.rotate * Constants.Swerve.MAX_ANGULAR_RATE
                )
            )
        }
    }
}