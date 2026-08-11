package org.chillout1778.subsystems

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants
import org.chillout1778.main
import org.chillout1778.subsystems.Arm.Shoulder.ClampWithinShoulderRange
import org.chillout1778.subsystems.Arm.Wrist.ClampWithinWristRange

/* REVISION:
Arm is the subsystem containing the WRIST and SHOULDER
*/
object Arm {

    fun Angle.asWPIAngle(): edu.wpi.first.units.measure.Angle {
        val asRad = this.inRad
        return edu.wpi.first.units.measure.Angle.ofBaseUnits(asRad, edu.wpi.first.units.Units.Radians)
    }

    enum class ArmPositions(val shoulder: Angle, val wrist: Angle) { // TODO("TUNE MEEEEEEEEEEEEEE!!!)
        Down(0.0.deg, 0.0.deg),
        SubStation(0.0.deg, 0.0.deg),
        L1Cone(0.0.deg, 0.0.deg),
        L2Cone(0.0.deg, 0.0.deg),
        L3Cone(0.0.deg, 0.0.deg),
        L1Cube(0.0.deg, 0.0.deg),
        L2Cube(0.0.deg, 0.0.deg),
        L3Cube(0.0.deg, 0.0.deg),
        GroundPickupForward(0.0.deg, 0.0.deg),
        GroundPickupBackward(0.0.deg, 0.0.deg)
        ;
    }

    fun goTo(positions: ArmPositions) {
        var shoulderPos = positions.shoulder.inRad
        var wristPos = positions.wrist.inRad


        // fancy stuff in here to make it not hit itself

        Shoulder.targetPosition = positions.shoulder.ClampWithinShoulderRange()
        Wrist.targetPosition = positions.wrist.ClampWithinWristRange()
    }

    object Shoulder : SubsystemBase() {

        var targetPosition = 0.0.deg
        private val mainMotor = TalonFX(Constants.CanIds.ARM_SHOULDER_MAIN_MOTOR, Constants.CanBusses.ARM_AND_MANIPULATOR).apply{
            configurator.apply(Constants.Shoulder.MOTOR_CONFIG)
        }

        private val followerMotor = TalonFX(Constants.CanIds.ARM_SHOULDER_FOLLOWER_MOTOR, Constants.CanBusses.ARM_AND_MANIPULATOR).apply{
            setControl(Follower(mainMotor.deviceID, false))
        }
        var isZeroed = false
        fun zero() {
            mainMotor.setPosition(0.0)
            isZeroed = true
        }


        fun Angle.ClampWithinShoulderRange(): Angle {
            return this
        }

        override fun periodic() {
            mainMotor.setControl(MotionMagicVoltage(targetPosition.asWPIAngle()))
        }

    }

    object Wrist : SubsystemBase() {

        var targetPosition = 0.0.deg
        private val mainMotor = TalonFX(Constants.CanIds.ARM_WRIST_MOTOR, Constants.CanBusses.ARM_AND_MANIPULATOR).apply {
            configurator.apply(Constants.Wrist.MOTOR_CONFIG)
        }

        fun Angle.ClampWithinWristRange(): Angle {
            return this
        }

        override fun periodic() {
            mainMotor.setControl(MotionMagicVoltage(targetPosition.asWPIAngle()))
        }


    }
}
