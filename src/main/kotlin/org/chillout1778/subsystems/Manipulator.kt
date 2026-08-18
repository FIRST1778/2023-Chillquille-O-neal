package org.chillout1778.subsystems

import com.ctre.phoenix6.hardware.TalonFX
import org.chillout1778.Constants
import org.wpilib.command2.SubsystemBase

object Manipulator : SubsystemBase() {

    enum class RollerState(val dutyCycle: Double) {

    }

    val rollerMotor = TalonFX(Constants.CanIds.MANIPULATOR_ROLLER_MAIN_MOTOR, Constants.CanBusses.MANIPULATOR)

}
