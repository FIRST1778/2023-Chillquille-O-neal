package org.chillout1778.subsystems

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants

object Manipulator : SubsystemBase() {

    enum class RollerState(val dutyCycle: Double) {

    }

    val rollerMotor = TalonFX(Constants.CanIds.MANIPULATOR_ROLLER_MAIN_MOTOR, Constants.CanBusses.ARM_AND_MANIPULATOR)

}
