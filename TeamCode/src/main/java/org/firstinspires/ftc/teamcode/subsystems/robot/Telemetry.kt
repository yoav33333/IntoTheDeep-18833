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
import org.firstinspires.ftc.teamcode.subsystems.BulkReads
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.deposit
import org.firstinspires.ftc.teamcode.subsystems.extendoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.linearSlides
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.getPose
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.magneticLimit
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.target
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
            telemetry,
            FtcDashboard.getInstance().telemetry
        )
    }
    var lastRunTime = 0.0

    /**
     * Resets the last recorded runtime to zero at the start of the user operation mode.
     *
     * This prepares the telemetry system to accurately track elapsed time during the operation mode loop.
     */
    override fun postUserStartHook(opMode: Wrapper) {
        lastRunTime = 0.0
    }
    /**
     * Collects and reports telemetry data from various robot subsystems and hardware components during each operation mode loop.
     *
     * This function updates the dashboard telemetry with current readings, servo positions, motor powers and positions, sensor states, scheduling status, and pose-related information to provide real-time monitoring of the robot's state.
     */
    override fun postUserLoopHook(opMode: Wrapper) {
        dashboardTelemetry.addData("current Control", BulkReads.modules[0].getCurrent(CurrentUnit.AMPS))
        dashboardTelemetry.addData("current Expention", BulkReads.modules[1].getCurrent(CurrentUnit.AMPS))
        dashboardTelemetry.addData("delta time", FeatureRegistrar.activeOpMode.runtime - lastRunTime)
        lastRunTime = FeatureRegistrar.activeOpMode.runtime
        dashboardTelemetry.addData("clawPosDepo", deposit.depoClawServo.position)
        dashboardTelemetry.addData("clawPos", clawSubsystem.clawServo.position)
        dashboardTelemetry.addData("v4b", armClawSubsystem.armClawServo.position)
        dashboardTelemetry.addData("v4b flip", armClawSubsystem.angleClawServo.position)
        dashboardTelemetry.addData("ex r", extendoSubsystem.extendoServoR.position)
        dashboardTelemetry.addData("ex l", extendoSubsystem.extendoServoL.position)
        dashboardTelemetry.addData("offset", linearSlides.offset)
        dashboardTelemetry.addData("sensor", magneticLimit.state)
        dashboardTelemetry.addData("sch", Mercurial.isScheduled(linearSlides.runToPosition))
        dashboardTelemetry.addData("leftCenter", linearSlides.leftCenter.power)
        dashboardTelemetry.addData("leftSide", linearSlides.leftSide.power)
        dashboardTelemetry.addData("rightSide", linearSlides.rightSide.power)
        dashboardTelemetry.addData("rightCenter", linearSlides.rightCenter.power)
        dashboardTelemetry.addData("leftCenterPose", linearSlides.leftCenter.currentPosition)
        dashboardTelemetry.addData("leftSidePose", linearSlides.leftSide.currentPosition)
        dashboardTelemetry.addData("rightSidePose", linearSlides.rightSide.currentPosition)
        dashboardTelemetry.addData("rightCenterPose", linearSlides.rightCenter.currentPosition)
        dashboardTelemetry.addData("rotate", clawSubsystem.clawRotationServo.position)
        dashboardTelemetry.addData("pose", getPose())
        dashboardTelemetry.addData("target", target)
        dashboardTelemetry.addData("error", target - getPose())
        dashboardTelemetry.addData("deposit claw", deposit.depoClawServo.position)
//      dashboardTelemetry.addData("diff", linearSlides.getPoseRight()-linearSlides.getPoseLeft())
        dashboardTelemetry.update()
    }
}