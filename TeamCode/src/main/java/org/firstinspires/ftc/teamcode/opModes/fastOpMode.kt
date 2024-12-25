//package org.firstinspires.ftc.teamcode.opModes
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import dev.frozenmilk.dairy.core.FeatureRegistrar
//import dev.frozenmilk.dairy.core.FeatureRegistrar._activeFeatures
//import dev.frozenmilk.dairy.core.FeatureRegistrar.opModeState
//import dev.frozenmilk.dairy.core.FeatureRegistrar.resolveRegistrationQueue
//import dev.frozenmilk.dairy.core.wrapper.Wrapper
//import dev.frozenmilk.dairy.core.wrapper.Wrapper.OpModeState
//import dev.frozenmilk.dairy.pasteurized.Pasteurized
//import dev.frozenmilk.mercurial.Mercurial
//import dev.frozenmilk.mercurial.bindings.Binding
//import dev.frozenmilk.mercurial.bindings.BoundGamepad
//import dev.frozenmilk.mercurial.commands.Command
//import dev.frozenmilk.mercurial.commands.MercurialException
//import dev.frozenmilk.mercurial.commands.UnwindCommandStack
//import dev.frozenmilk.util.cell.LazyCell
//import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode
//import java.util.Collections
//import java.util.WeakHashMap
//
//open class fastOpMode : LinearOpMode() {
//    var state = OpModeState.INIT
//
//    override fun runOpMode() {
//        runInit()
//        runLoop()
//    }
//
//    fun runInit(){
//        state = OpModeState.INIT
//        startInit()
//        while (opModeInInit()){
//            startLoop()
//            initLoop()
//            finishLoop()
//        }
//    }
//
//    fun runLoop(){
//        state = OpModeState.ACTIVE
//        while (opModeIsActive()){
//
//            startLoop()
//            actualLoop()
//            FeatureRegistrar.activeFeatures.reversed().forEach { it.postUserLoopHook(Wrapper) }
//        }
//    }
//    open fun startInit(){}
//    open fun initLoop(){}
//
//    open fun actualLoop(){}
//
//}