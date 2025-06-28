package org.firstinspires.ftc.teamcode.subsystems.intakeClaw

import com.acmerobotics.dashboard.config.Config

@Config
object IntakeClawVariables {
    @JvmField var closedClawPosition = 0.28
    @JvmField var openedClawPosition = 0.0
    @JvmField var centeredRotation = 0.28
    @JvmField var rotationSpeed = 0.13
    @JvmField var leftRotation = 0.7
    @JvmField var flipRotation = centeredRotation
}