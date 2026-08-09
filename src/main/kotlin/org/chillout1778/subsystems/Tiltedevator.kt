package org.chillout1778.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.util.Units
import kotlin.math.abs

object Elevator: SubsystemBase() {
    enum class State(val extension: Double) {
        //none of these values are right. must check CAD and then tune.
        Down(extension = 0.0),
        SubStation(extension = Units.inchesToMeters(1000.0)),
        L1Cone(extension = Units.inchesToMeters(1000.0)),
        L2Cone(extension = Units.inchesToMeters(1000.0)),
        L3Cone(extension = Units.inchesToMeters(1000.0)),
        L1Cube(extension = Units.inchesToMeters(1000.0)),
        L2Cube(extension = Units.inchesToMeters(1000.0)),
        L3Cube(extension = Units.inchesToMeters(1000.0)),
        GroundPickupForward(extension = Units.inchesToMeters(1000.0)),
        GroundPickupBackward(extension = Units.inchesToMeters(1000.0)),
        ;

    }

    private val mainMotor = TalonFX(Constants.CanIds.ELEVATOR_MAIN_MOTOR).apply{
        configurator.apply(Constants.Elevator.MOTOR_CONFIG)
    }

    private val followerMotor = TalonFX(Constants.CanIds.ELEVATOR_FOLLOWER_MOTOR).apply{
        setControl(Follower(mainMotor.deviceID, true))
    }

    var isZeroed = false
    fun zero() {
        mainMotor.setPosition(0.0)
        isZeroed = true
    }

    var state = State.Down

    val height get() = mainMotor.position.valueAsDouble
    val velocity get() = mainMotor.velocity.valueAsDouble

    val atSetPoint get() = abs(height - state.extension) < Constants.Elevator.SETPOINT_THRESHOLD
    val lazierAtSetPoint get() = abs(height - state.extension) < Constants.Elevator.LAZIER_SETPOINT_THRESHOLD
    val atOrAboveSetPoint get() = (height + Constants.Elevator.SETPOINT_THRESHOLD) >= state.extension

    override fun periodic() {
        if (!isZeroed || !Arm.Shoulder.isZeroed)
            return
        mainMotor.setControl(MotionMagicVoltage(state.extension))
    }
    fun setZeroingVoltage() {
        mainMotor.setVoltage(Constants.Elevator.ZERO_VOLTAGE)
    }

    fun stop(){
        mainMotor.setVoltage(0.0)
    }

    fun setCoastEnabled(coast: Boolean) {
        if (coast) {
            mainMotor.setNeutralMode(NeutralModeValue.Coast)
            followerMotor.setNeutralMode(NeutralModeValue.Coast)
        } else {
            mainMotor.setNeutralMode(NeutralModeValue.Brake)
            followerMotor.setNeutralMode(NeutralModeValue.Brake)
        }
    }
}
