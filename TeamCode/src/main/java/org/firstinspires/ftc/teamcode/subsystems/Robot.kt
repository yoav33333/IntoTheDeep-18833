package org.firstinspires.ftc.teamcode.subsystems

import com.pedropathing.localization.Pose

object Robot {
    val intakingColors = mapOf("alliance" to null, "additional" to intakeSubsystem.Color.YELLOW)

    fun updateAllianceColor(color: intakeSubsystem.Color?) {
        intakingColors["alliance"] to color
    }
    fun updateAdditionalColor(color: intakeSubsystem.Color?) {
        intakingColors["additional"] to color
    }
    fun intakeYellows() {
        updateAdditionalColor(intakeSubsystem.Color.YELLOW)
    }
    fun intakeOnlyAlliance() {
        updateAllianceColor(null)
    }
    val currentPose = Pose(0.0,0.0,0.0)
}