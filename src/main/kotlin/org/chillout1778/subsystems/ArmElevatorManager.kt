package org.chillout1778.org.chillout1778.subsystems

import edu.wpi.first.wpilibj.RobotState
import org.chillout1778.org.chillout1778.Angle
import org.chillout1778.org.chillout1778.Distance
import org.chillout1778.org.chillout1778.deg
import org.chillout1778.org.chillout1778.m
import org.chillout1778.org.chillout1778.rev
import org.chillout1778.subsystems.Arm.Shoulder
import org.chillout1778.subsystems.Arm.Shoulder.ClampWithinShoulderRange
import org.chillout1778.subsystems.Arm.Wrist
import org.chillout1778.subsystems.Arm.Wrist.ClampWithinWristRange
import org.chillout1778.subsystems.Elevator
import org.chillout1778.subsystems.Elevator.ClampWithinElevatorRange


object ArmElevatorManager {

    data class robotConfiguration(var elevator: Distance, var shoulder: Angle, var wrist: Angle)

    enum class SystemStates(val robotState: robotConfiguration) { // TODO("TUNE MEEEEEEEEEEEEEE!!!)
        Down(robotConfiguration(0.0.m, 0.0.deg, 0.0.deg)),
        SubStation(robotConfiguration(0.0.m, 0.0.deg, 0.0.deg)),
        L1Cone(robotConfiguration(0.0.m, 0.0.deg, 0.0.deg)),
        L2Cone(robotConfiguration(0.0.m, 0.0.deg, 0.0.deg)),
        L3Cone(robotConfiguration(0.0.m, 0.0.deg, 0.0.deg)),
        L1Cube(robotConfiguration(0.0.m, 0.0.deg, 0.0.deg)),
        L2Cube(robotConfiguration(0.0.m, 0.0.deg, 0.0.deg)),
        L3Cube(robotConfiguration(0.0.m, 0.0.deg, 0.0.deg)),
        GroundPickupForward(robotConfiguration(0.0.m, 0.0.deg, 0.0.deg)),
        GroundPickupBackward(robotConfiguration(0.0.m, 0.0.deg, 0.0.deg))
        ;
    }

    fun goTo(position: robotConfiguration) {
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

    var currentState = SystemStates.Down.robotState
    var targetState = SystemStates.Down.robotState

    fun update() {
        currentState = robotConfiguration(Elevator.height.m, Shoulder.mainMotor.position.valueAsDouble.rev, Wrist.mainMotor.position.valueAsDouble.rev)
        goTo(targetState)
    }
}

fun Angle.asWPIAngle(): edu.wpi.first.units.measure.Angle {
    val asRad = this.inRad
    return edu.wpi.first.units.measure.Angle.ofBaseUnits(asRad, edu.wpi.first.units.Units.Radians)
}