package org.chillout1778.commands

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.wpilibj2.command.Command
import java.util.function.Supplier
import org.chillout1778.Controls.DriveInputs
import org.chillout1778.subsystems.Swerve
import org.chillout1778.Constants

class TeleopDriveCommand (
    private val driveInputsSupplier: Supplier<DriveInputs>
    ): Command() {
    init {
        addRequirements(Swerve)
    }

    val drive = SwerveRequest.FieldCentric()
        .withDeadband(Constants.Swerve.MAX_SPEED * 0.1)
        .withRotationalDeadband(Constants.Swerve.MAX_SPEED * 0.1)
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)

    val brake = SwerveRequest.SwerveDriveBrake()

    override fun execute() {
        val inputs = driveInputsSupplier.get()

        Swerve.setControl(
            if (driveInputsSupplier.get().wantSwerveBrake) brake
            else
                drive.withVelocityX(inputs.axial * Constants.Swerve.MAX_SPEED)
                    .withVelocityY(inputs.lateral * Constants.Swerve.MAX_SPEED)
                    .withRotationalRate(inputs.rotate * Constants.Swerve.MAX_ANGULAR_RATE)

        )
    }


}