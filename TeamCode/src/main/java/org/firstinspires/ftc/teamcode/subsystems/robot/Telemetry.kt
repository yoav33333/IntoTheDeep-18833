package org.firstinspires.ftc.teamcode.subsystems.robot

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawHardware
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftHardware
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftHardware.getPose
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftHardware.magneticLimit
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.targetPosition

import java.lang.annotation.Inherited

object Telemetry: SDKSubsystem() {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    val dashboardTelemetry: MultipleTelemetry by OpModeLazyCell {
        MultipleTelemetry(
            FeatureRegistrar.activeOpMode.telemetry,
            FtcDashboard.getInstance().telemetry
        )
    }
    var lastRunTime = 0.0

    override fun postUserStartHook(opMode: Wrapper) {
        lastRunTime = 0.0
    }
    override fun postUserLoopHook(opMode: Wrapper) {
        dashboardTelemetry.addData("current Control", BulkReads.modules[0].getCurrent(CurrentUnit.AMPS))
        dashboardTelemetry.addData("current Expention", BulkReads.modules[1].getCurrent(CurrentUnit.AMPS))
        dashboardTelemetry.addData("delta time", FeatureRegistrar.activeOpMode.runtime - lastRunTime)
        lastRunTime = FeatureRegistrar.activeOpMode.runtime
        dashboardTelemetry.addData("offset", LiftVariables.offset)
        dashboardTelemetry.addData("sensor", magneticLimit.state)
        dashboardTelemetry.addData("leftCenter", LiftHardware.leftCenter.power)
        dashboardTelemetry.addData("leftSide", LiftHardware.leftSide.power)
        dashboardTelemetry.addData("rightSide", LiftHardware.rightSide.power)
        dashboardTelemetry.addData("rightCenter", LiftHardware.rightCenter.power)
        dashboardTelemetry.addData("leftCenterPose", LiftHardware.leftCenter.currentPosition)
        dashboardTelemetry.addData("leftSidePose", LiftHardware.leftSide.currentPosition)
        dashboardTelemetry.addData("rightSidePose", LiftHardware.rightSide.currentPosition)
        dashboardTelemetry.addData("rightCenterPose", LiftHardware.rightCenter.currentPosition)
        dashboardTelemetry.addData("rotate", IntakeClawHardware.rotationServo.position)
        dashboardTelemetry.addData("pose", getPose())
        dashboardTelemetry.addData("target", targetPosition)
        dashboardTelemetry.addData("error", targetPosition - getPose())


      dashboardTelemetry.addData("lift state", LiftVariables.liftState.name)
        dashboardTelemetry.update()
    }
}