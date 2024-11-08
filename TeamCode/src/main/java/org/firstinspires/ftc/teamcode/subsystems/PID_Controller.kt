package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotorEx
import dev.frozenmilk.dairy.core.util.controller.calculation.ControllerCalculation
import dev.frozenmilk.dairy.core.util.controller.calculation.ControllerComponent
import dev.frozenmilk.dairy.core.util.controller.calculation.logical.eval
import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent
import dev.frozenmilk.dairy.core.util.controller.calculation.pid.UnitComponent
import dev.frozenmilk.dairy.core.util.controller.implementation.DistancePoseController
import dev.frozenmilk.dairy.core.util.controller.implementation.DistanceVectorController
import dev.frozenmilk.dairy.core.util.controller.implementation.DoubleController
import dev.frozenmilk.dairy.core.util.controller.implementation.DoublePoseController
import dev.frozenmilk.dairy.core.util.controller.implementation.DoubleVectorController
import dev.frozenmilk.dairy.core.util.controller.implementation.UnitController
import dev.frozenmilk.dairy.core.util.supplier.numeric.CachedMotionComponentSupplier
import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier
import dev.frozenmilk.dairy.core.util.supplier.numeric.MotionComponentSupplier
import dev.frozenmilk.dairy.core.util.supplier.numeric.MotionComponents
import dev.frozenmilk.dairy.core.util.supplier.numeric.unit.EnhancedUnitSupplier
import dev.frozenmilk.util.units.distance.Distance
import dev.frozenmilk.util.units.distance.DistanceUnits
import dev.frozenmilk.util.units.distance.cm
import dev.frozenmilk.util.units.distance.ft
import dev.frozenmilk.util.units.distance.inch
import dev.frozenmilk.util.units.distance.m
import dev.frozenmilk.util.units.distance.mm
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import java.util.EnumMap

class PID_Controller(val currentPose: EnhancedDoubleSupplier,val targetPose: EnhancedDoubleSupplier, val p: Double, val i: Double, val d: Double){

    private var doubleControllerTarget = 0.0
    private var unitControllerTarget = Distance()

    //
    // Controllers
    //
    // ComplexControllers are the Dairy alternative to run modes, allowing for powerful, extensible control loops
//        doubleControllerTarget = target // we'll use this to control the target
    val doubleController = DoubleController(
        // target
        // NaN can be returned for a component if you want to completely ignore it
        // but usually something else is better: 0.0, NEGATIVE_INFINITY, POSITIVE_INFINITY
        // in this case we're only ever going to use state for a calculation
        targetSupplier = MotionComponentSupplier {
            return@MotionComponentSupplier targetPose.state},
        // state
        // we'll use the motor's encoder for feedback
        stateSupplier = currentPose,
        // tolerance
        // when we check if we're finished, this is our default allowable error
        // NaN can be returned for a component if you want to completely ignore it
        // this cached wrapper will prevent regenerating the outputs, as they aren't dynamic
        toleranceEpsilon = CachedMotionComponentSupplier(
            MotionComponentSupplier {
                return@MotionComponentSupplier when (it) {
                    MotionComponents.STATE -> 10.0
                    MotionComponents.VELOCITY -> 1.0
                    else -> Double.NaN
                }
            }
        ),
        // optional, callback
//            outputConsumer = motor::setPower, // when this controller updates, this callback will be run
        // then we build up the calculation:
        controllerCalculation = DoubleComponent.P(MotionComponents.STATE, p) // first P
            .plus(DoubleComponent.I(MotionComponents.STATE, i, -0.1, 0.1)) // then I
            .plus(DoubleComponent.D(MotionComponents.STATE, d)) // then D
    )





}