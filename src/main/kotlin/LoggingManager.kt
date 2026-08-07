package org.chillout1778

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotBase
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGWriter

object LoggingManager {
    fun start() {
        // 1. Record build/git metadata into log files
        Logger.recordMetadata("ProjectName", "Chillquille-SystemCore")
        Logger.recordMetadata("BuildDate", "2026")

        // 2. Configure output sinks depending on execution mode
        if (RobotBase.isReal()) {
            // Write continuous binary log files onto SystemCore local storage (/home/lvuser or USB)
            Logger.addDataReceiver(WPILOGWriter())

            // Stream values live over NetworkTables (to AdvantageScope / Elastic)
            Logger.addDataReceiver(NT4Publisher())

            // Optionally register Power Distribution Panel (REV PDH or CTRE PDP)
            // PowerDistribution(1, PowerDistribution.ModuleType.kRev)
        } else {
            // In Desktop Simulation Mode
            Logger.addDataReceiver(NT4Publisher())
        }

        // 3. Start the AdvantageKit logging engine thread
        Logger.start()
    }
}