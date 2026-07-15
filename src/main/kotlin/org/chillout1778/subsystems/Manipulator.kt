package org.chillout1778.subsystems

object Manipulator : SubsystemBase() {

    enum class RollerState(val dutyCycle: Double) {

    }

    val rollerMotor = TalonFX(Constant)

}
