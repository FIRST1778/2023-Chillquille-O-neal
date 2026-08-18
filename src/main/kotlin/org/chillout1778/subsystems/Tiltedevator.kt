package org.chillout1778.subsystems

import org.chillout1778.Constants
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import org.chillout1778.org.chillout1778.Distance
import org.chillout1778.org.chillout1778.m
import org.chillout1778.org.chillout1778.subsystems.ArmElevatorManager.currentState
import org.wpilib.command2.SubsystemBase
import kotlin.math.abs

object Elevator: SubsystemBase() {

    val mainMotor = TalonFX(Constants.CanIds.ELEVATOR_MAIN_MOTOR, Constants.CanBusses.ELEVATOR).apply{
        configurator.apply(Constants.Elevator.MOTOR_CONFIG)
    }

    private val followerMotor = TalonFX(Constants.CanIds.ELEVATOR_FOLLOWER_MOTOR, Constants.CanBusses.ELEVATOR).apply{
        setControl(Follower(mainMotor.deviceID, MotorAlignmentValue.Opposed))
    }

    var isZeroed = false
    fun zero() {
        mainMotor.setPosition(0.0)
        isZeroed = true
    }

    var targetPosition = 0.0.m

    val height get() = mainMotor.position.valueAsDouble
    val velocity get() = mainMotor.velocity.valueAsDouble

    val atSetPoint get() = abs(height - targetPosition.inMeters) < Constants.Elevator.SETPOINT_THRESHOLD
    val lazierAtSetPoint get() = abs(height - targetPosition.inMeters) < Constants.Elevator.LAZIER_SETPOINT_THRESHOLD
    val atOrAboveSetPoint get() = (height + Constants.Elevator.SETPOINT_THRESHOLD) >= targetPosition.inMeters

    override fun periodic() {
        if (!isZeroed || !Arm.Shoulder.isZeroed)
            return
        mainMotor.setControl(MotionMagicVoltage(targetPosition.inMeters))
    }
    fun setZeroingVoltage() {
        mainMotor.setVoltage(Constants.Elevator.ZERO_VOLTAGE)
    }

    fun stop(){
        mainMotor.setVoltage(0.0)
    }

    fun setCoastEnabled(coast: Boolean) {
        if (coast) {
            mainMotor.configNeutralMode(NeutralModeValue.Coast)
            followerMotor.configNeutralMode(NeutralModeValue.Coast)
        } else {
            mainMotor.configNeutralMode(NeutralModeValue.Brake)
            followerMotor.configNeutralMode(NeutralModeValue.Brake)
        }
    }

    fun Distance.ClampWithinElevatorRange(): Distance {
        return this.inMeters.coerceIn(0.0, 1.0).m // TODO("tune me")
    }
}
