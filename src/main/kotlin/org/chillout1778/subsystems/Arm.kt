package org.chillout1778.subsystems

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants
import org.chillout1778.org.chillout1778.Angle
import org.chillout1778.org.chillout1778.deg
import org.chillout1778.org.chillout1778.rad
import org.chillout1778.org.chillout1778.subsystems.asWPIAngle
import kotlin.math.PI

/* REVISION:
Arm is the subsystem containing the WRIST and SHOULDER
*/
object Arm {


    object Shoulder : SubsystemBase() {

        var targetPosition = 0.0.deg
        val mainMotor = TalonFX(Constants.CanIds.ARM_SHOULDER_MAIN_MOTOR, Constants.CanBusses.ARM).apply{
            configurator.apply(Constants.Shoulder.MOTOR_CONFIG)
        }

        private val followerMotor = TalonFX(Constants.CanIds.ARM_SHOULDER_FOLLOWER_MOTOR, Constants.CanBusses.ARM).apply{
            setControl(Follower(mainMotor.deviceID, false))
        }
        var isZeroed = false
        fun zero() {
            mainMotor.setPosition(0.0)
            isZeroed = true
        }


        fun Angle.ClampWithinShoulderRange(): Angle { // TODO("tune")
            return this.inRad.coerceIn(0.0, 2*PI).rad // Converts from angle to radians, then makes sure its within range. then it converts back to angle
        }

        override fun periodic() {
            mainMotor.setControl(MotionMagicVoltage(targetPosition.asWPIAngle()))
        }

    }

    object Wrist : SubsystemBase() {

        var targetPosition = 0.0.deg
        val mainMotor = TalonFX(Constants.CanIds.ARM_WRIST_MOTOR, Constants.CanBusses.ARM).apply {
            configurator.apply(Constants.Wrist.MOTOR_CONFIG)
        }

        fun Angle.ClampWithinWristRange(): Angle { // TODO("tune")
            return this.inRad.coerceIn(0.0, 2*PI).rad // Converts from angle to radians, then makes sure its within range. then it converts back to angle
        }

        override fun periodic() {
            mainMotor.setControl(MotionMagicVoltage(targetPosition.asWPIAngle()))
        }
    }
}
