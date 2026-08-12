package org.chillout1778.org.chillout1778.subsystems

import org.chillout1778.org.chillout1778.Angle
import org.chillout1778.org.chillout1778.Distance
import org.chillout1778.org.chillout1778.deg
import org.chillout1778.org.chillout1778.m
import org.chillout1778.subsystems.Arm.Shoulder
import org.chillout1778.subsystems.Arm.Shoulder.ClampWithinShoulderRange
import org.chillout1778.subsystems.Arm.Wrist
import org.chillout1778.subsystems.Arm.Wrist.ClampWithinWristRange
import org.chillout1778.subsystems.Elevator
import org.chillout1778.subsystems.Elevator.ClampWithinElevatorRange


object ArmElevatorManager {

    enum class SystemStates(val elevator: Distance, val shoulder: Angle, val wrist: Angle) { // TODO("TUNE MEEEEEEEEEEEEEE!!!)
        Down(0.0.m, 0.0.deg, 0.0.deg),
        SubStation(0.0.m, 0.0.deg, 0.0.deg),
        L1Cone(0.0.m, 0.0.deg, 0.0.deg),
        L2Cone(0.0.m, 0.0.deg, 0.0.deg),
        L3Cone(0.0.m, 0.0.deg, 0.0.deg),
        L1Cube(0.0.m, 0.0.deg, 0.0.deg),
        L2Cube(0.0.m, 0.0.deg, 0.0.deg),
        L3Cube(0.0.m, 0.0.deg, 0.0.deg),
        GroundPickupForward(0.0.m, 0.0.deg, 0.0.deg),
        GroundPickupBackward(0.0.m, 0.0.deg, 0.0.deg)
        ;
    }

    fun goTo(position: SystemStates) {
        var shoulderPos = position.shoulder.inRad
        var wristPos = position.wrist.inRad
        var elevatorPos = position.elevator.inMeters


        /**
         * Probably going to do something here with a LUT? A linear function? IDK
         * but it will need to know where it is safe to have the arm and manip at a given elevator height.
         **/

        Shoulder.targetPosition = shoulderPos.deg.ClampWithinShoulderRange()
        Wrist.targetPosition = wristPos.deg.ClampWithinWristRange()
        Elevator.targetPosition = elevatorPos.m.ClampWithinElevatorRange()
    }

    var currentState = SystemStates.Down
    var targetState = SystemStates.Down

    fun update() {
        goTo(currentState)
    }
}

fun Angle.asWPIAngle(): edu.wpi.first.units.measure.Angle {
    val asRad = this.inRad
    return edu.wpi.first.units.measure.Angle.ofBaseUnits(asRad, edu.wpi.first.units.Units.Radians)
}