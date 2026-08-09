package org.chillout1778.subsystems

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants

/* REVISION:
Arm is the subsystem containing the WRIST and SHOULDER
*/
object Arm {
    object Shoulder : SubsystemBase() {
        private val mainMotor = TalonFX(Constants.CanIds.ARM_SHOULDER_MAIN_MOTOR).apply{
            configurator.apply(Constants.Shoulder.MOTOR_CONFIG)
        }

        private val followerMotor = TalonFX(Constants.CanIds.ARM_SHOULDER_FOLLOWER_MOTOR).apply{
            setControl(Follower(mainMotor.deviceID, false))
        }
        var isZeroed = false
        fun zero() {
            mainMotor.setPosition(0.0)
            isZeroed = true
        }

    }

    object Wrist : SubsystemBase() {
        private val mainMotor = TalonFX(Constants.CanIds.ARM_WRIST_MOTOR).apply {
            configurator.apply(Constants.Wrist.MOTOR_CONFIG)
        }

        fun Double.ClampWithinRange(): Double {
            return this
        }


    }
}
