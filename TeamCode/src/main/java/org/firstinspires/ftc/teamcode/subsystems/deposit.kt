package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.cachinghardware.CachingServo
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.dairy.pasteurized.Pasteurized
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta.Flavor
import org.firstinspires.ftc.teamcode.commands.extendoCommand
import org.firstinspires.ftc.teamcode.commands.util.WaitUntil
import java.lang.annotation.Inherited
@Config
object deposit : SDKSubsystem() {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    val depoClawServo: CachingServo by OpModeLazyCell {
        val s = CachingServo(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "small claw"
            )
        )
        s
    }

    val depoArmServo: CachingServo by OpModeLazyCell {
        val s = CachingServo(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "flip servo"
            )
        )
        s
    }
    val colorSensor: RevColorSensorV3 by OpModeLazyCell {
        val s = FeatureRegistrar.activeOpMode.hardwareMap.get(
            RevColorSensorV3::class.java, "dsc"
        )
        s
    }
    var isSpe = false
    val closeingClawPose = 0.0





        val openingClawPose = 1.0
    @JvmField
    var ArmInPose = 0.01
    @JvmField
    var ArmHalfInPose = 0.1
    @JvmField
    var almostArmInPose = 0.069
//    @JvmField
    var ArmOutPose = 0.71
    @JvmField
    var ArmOutPoseChamber = 0.86
    val ArmOutPoseParallel = 0.8
    val ArmOutPose2 = 0.82
    var isArmOut = true
    val armMaybeOut = Lambda("amo")
        .setInit{
            if (isArmOut){

                armOut.schedule()
            }
            else{
                armOutHalf()
            }
        }
    val switchArmOut = Lambda("sao")
        .setInit{
            isArmOut = !isArmOut
//            if (depoArmServo.position == ArmOutPose){
            armMaybeOut.schedule()
        }
    /**
     * Closes the deposit claw by setting the servo to the closed position.
     */
    @JvmStatic
    fun closeClaw() {
        depoClawServo.setPosition(closeingClawPose)
    }
    @JvmStatic
    fun closeClawRaw() {
        depoClawServo.setPositionRaw(closeingClawPose)
    }

    fun openClaw() {
        depoClawServo.setPosition(openingClawPose)
    }

    fun armIn() {
        depoArmServo.setPosition(ArmInPose)
    }
    /**
     * Moves the deposit arm to the out position, selecting a special chamber pose if in TELEOP mode and `isSpe` is true.
     */
    @JvmStatic

    fun armOut() {
        depoArmServo.setPosition(if (isSpe && FeatureRegistrar.activeOpModeWrapper.opModeType == Flavor.TELEOP) ArmOutPoseChamber else ArmOutPose)
    }
    @JvmStatic

    val armOutBasket = Lambda("aob")
        .setInit{
        depoArmServo.setPosition(ArmOutPoseParallel)
    }
    /**
     * Moves the deposit arm servo to the half-in position using the configured `ArmHalfInPose` value.
     */
    @JvmStatic
    fun armOutHalf() {
        depoArmServo.setPosition(ArmHalfInPose)
    }

    /**
     * Moves the deposit arm to the secondary out position.
     *
     * Sets the arm servo to the position defined by `ArmOutPose2`.
     */
    fun armOut2() {
        depoArmServo.setPosition(ArmOutPose2)
    }

    fun transferPose() {
        openClaw()
        armIn()
    }

    fun intakeFromHumanPlayer() {
        openClaw()
        armOut2()
    }

    fun depoPreset() {
        closeClaw()
        armOut()
    }

    /**
     * Checks if a sample is detected within 33 mm by the color sensor and closes the claw if detected.
     *
     * If gamepad2 button A is pressed, restarts the intake sequence. Returns true if an object is detected within range; otherwise, returns false.
     *
     * @return True if a sample is detected and the claw is closed; false otherwise.
     */
    fun checkIfSampleInPlace(): Boolean {
        if (Pasteurized.gamepad2.a.onTrue){
            intakeSeq.cancel()
            intakeSeq.schedule()
        }

        if (colorSensor.getDistance(DistanceUnit.MM) < 33) {
            closeClaw()
            return true
        }
        return false
    }

    val slamArm = Lambda("slamArm")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            depoArmServo.position = 0.2
//            linearSlides.target += 13000
        }
    @JvmStatic
    val slamArmDown = Lambda("slamArm")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            depoArmServo.position = 0.9
            linearSlides.target -= 19000
            Sequential(Wait(0.3), release).schedule()

        }
    @JvmStatic
    val release = Lambda("release")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { openClaw() }
    val up = Lambda("up")
        .setInit{
            linearSlides.target += 19000
        }
    val releaseH = Lambda("Hrelease")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { depoClawServo.position = 0.4}
    val closeH = Lambda("close")
        .setInit{closeClaw()}
    val quickRC = Sequential(
        WaitUntil{ linearSlides.getPose()>1000},
        releaseH, Wait(0.5), closeH)
    @JvmStatic
    val quickRCSimple = Sequential(releaseH, Wait(0.3), closeH)
    @JvmStatic
    val slamSeq = Sequential(slamArm, Wait(0.0), up, Wait(0.15), release)
//    @JvmStatic
//    val slamSeqDown = Sequential(slamArmDown, Wait(0.1), release)
    val changeClawPos = Lambda("changeClawPos")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            if (depoClawServo.position == closeingClawPose) {
                if (isSpe) slamSeq.schedule()
                else openClaw()
            } else {
                closeClaw()
            }
        }

    @JvmStatic
    val armOut = Lambda("armOut")
        .setInit { armOut() }
    @JvmStatic
    val armIn = Lambda("armIn")
        .setInit { armIn() }
    var stop = false
    @JvmStatic
    val intakeCommand = Lambda("intakeCommand")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            intakeFromHumanPlayer()
            linearSlides.runToPosition.schedule()
            stop = false
            extendoCommand.extendoMacro.cancel()
            transferCommand.cancel()
        }
    @JvmStatic
    val catchPixel = Lambda("catchPixel")
        .setFinish {
            checkIfSampleInPlace() || stop
        }
        .setInterruptible{true}
    val catchSimple = Lambda("catchSimple")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { closeClaw() }
    @JvmStatic
    val postIntakeState = Lambda("postIntakeState")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { armOut() }
    val intakeSeq = Sequential(
        intakeCommand,
        Wait(0.5),
        catchPixel,
        Wait(0.3),
        postIntakeState
    )


    val transferCommand = Lambda("transferCommand")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            transferPose()
            stop = true

        }
        .setExecute {
            telemetry.addData("active", true)
        }
        .setFinish {
            checkIfSampleInPlace()
        }
    val halfArmIn = Lambda("HAI")
        .setInit{ depoArmServo.position = 0.43}

    val almostArmIn = Lambda("AAI")
        .setInit{ depoArmServo.position = almostArmInPose}
    val transferSeq = Sequential(
        transferCommand,
        Wait(0.05),
        clawSubsystem.openClaw,
        halfArmIn
    )
    val transferSeqAuto = Sequential(
        transferCommand.raceWith(Wait(1.0)),
        closeH,
//        Wait(0.1),
        clawSubsystem.openClaw,
        halfArmIn
    )
    val TransferState = Lambda("TransferState")
        .setInit {
            armIn()
            openClaw()
            stop = true
        }


    override fun postUserInitHook(opMode: Wrapper) {
//        armIn()
    }

}