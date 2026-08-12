package org.chillout1778.org.chillout1778

import kotlin.math.abs

data class Angle private constructor(override val value: Double) : Quantity<Angle>() {
    companion object {
        private const val DEGREES_TO_RADIANS = Math.PI / 180
        private const val REVOLUTIONS_TO_RADIANS = 2 * Math.PI

        private fun fromUnit(value: Double, scalar: Double) = Angle(value * scalar)

        /**
         * Creates a new [Angle] from an angle in radians
         * @param radians the angle in radians
         */
        @JvmStatic
        fun fromRad(radians: Double) = Angle(radians)

        /**
         * Creates a new [Angle] from an angle in radians
         * @param radians the angle in radians
         */
        @JvmStatic
        fun fromRad(radians: Int) = fromRad(radians.toDouble())

        /**
         * Creates a new [Angle] from an angle in degrees
         * @param degrees the angle in degrees
         */
        @JvmStatic
        fun fromDeg(degrees: Double) = fromUnit(degrees, DEGREES_TO_RADIANS)

        /**
         * Creates a new [Angle] from an angle in degrees
         * @param degrees the angle in degrees
         */
        @JvmStatic
        fun fromDeg(degrees: Int) = fromDeg(degrees.toDouble())

        /**
         * Creates a new [Angle] from an angle in full revolutions
         * @param revolutions the angle in full revolutions
         */
        @JvmStatic
        fun fromRev(revolutions: Double) = fromUnit(revolutions, REVOLUTIONS_TO_RADIANS)

        /**
         * Creates a new [Angle] from an angle in full revolutions
         * @param revolutions the angle in full revolutions
         */
        @JvmStatic
        fun fromRev(revolutions: Int) = fromRev(revolutions.toDouble())

        fun wrapAngle0To2Pi(angle: Double) = ((angle % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)
        fun wrapAnglePiToPi(angle: Double) =
            ((angle + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI
    }

    /**
     * The value of the angle in radians
     */
    @JvmField
    val inRad = value

    /**
     * The value of the angle in degrees
     */
    @JvmField
    val inDeg = value / DEGREES_TO_RADIANS

    /**
     * The value of the angle in full revolutions
     */
    @JvmField
    val inRev = value / REVOLUTIONS_TO_RADIANS

    /**
     * A new [Angle] wrapped from 0 to 2pi
     */
    @get:JvmName("wrapped")
    val wrapped get() = Angle(wrapAngle0To2Pi(value))

    /**
     * A new [Angle] wrapped from -pi to pi
     */
    @get:JvmName("normalized")
    val normalized get() = Angle(wrapAnglePiToPi(value))

    override fun newInstance(value: Double): Angle = Angle(value)

    override fun toString(): String = "$value rad"
}

/**
 * Creates a new [Angle] from an angle in radians
 */
val Double.rad: Angle get() = Angle.fromRad(this)

/**
 * Creates a new [Angle] from an angle in radians
 */
val Int.rad: Angle get() = Angle.fromRad(this)

/**
 * Creates a new [Angle] from an angle in degrees
 */
val Double.deg: Angle get() = Angle.fromDeg(this)

/**
 * Creates a new [Angle] from an angle in degrees
 */
val Int.deg: Angle get() = Angle.fromDeg(this)

/**
 * Creates a new [Angle] from an angle in full revolutions
 */
val Double.rev: Angle get() = Angle.fromRev(this)

/**
 * Creates a new [Angle] from an angle in full revolutions
 */
val Int.rev: Angle get() = Angle.fromRev(this)

/**
 * Represents a physical quantity
 * @param T the type of the quantity
 * @author BeepBot99
 */
abstract class Quantity<T : Quantity<T>> {
    /**
     * The value of the quantity
     */
    abstract val value: Double

    operator fun plus(other: T): T = newInstance(value + other.value)
    operator fun minus(other: T): T = newInstance(value - other.value)
    operator fun times(scalar: Double): T = newInstance(value * scalar)
    operator fun times(scalar: Int): T = newInstance(value * scalar)
    operator fun div(other: T): Double = value / other.value
    operator fun div(scalar: Double): T = newInstance(value / scalar)
    operator fun div(scalar: Int): T = newInstance(value / scalar)
    operator fun unaryPlus(): T = newInstance(value)
    operator fun unaryMinus(): T = newInstance(-value)
    operator fun rem(other: T): T = newInstance(value % other.value)
    operator fun rem(divisor: Double): T = newInstance(value % divisor)
    operator fun rem(divisor: Int): T = newInstance(value % divisor)
    operator fun compareTo(other: T): Int = value.compareTo(other.value)

    fun lessThan(other: T): Boolean = compareTo(other) < 0
    fun lessThanOrEqualTo(other: T): Boolean = compareTo(other) <= 0
    fun greaterThan(other: T): Boolean = compareTo(other) > 0
    fun greaterThanOrEqualTo(other: T): Boolean = compareTo(other) >= 0

    val sign: Int
        get() = when {
            value > 0 -> 1
            value < 0 -> -1
            else -> 0
        }

    @get:JvmName("abs")
    val abs: T get() = newInstance(abs(value))

    abstract override fun equals(other: Any?): Boolean
    abstract override fun hashCode(): Int
    abstract override fun toString(): String

    /**
     * @return if [value] is NaN
     */
    fun isNaN(): Boolean = value.isNaN()

    /**
     * Creates a new instance of the class with the given value
     * @param value the value to create an instance with
     */
    abstract fun newInstance(value: Double): T
}

fun <T : Quantity<T>> abs(quantity: T): T = quantity.abs

/**
 * A quantity of distance or length
 * @param value the distance in millimeters
 * @author BeepBot99
 */
data class Distance private constructor(override val value: Double) : Quantity<Distance>() {
    companion object {
        private const val CENTIMETERS_TO_MILLIMETERS = 10.0
        private const val METERS_TO_MILLIMETERS = 1000.0
        private const val INCHES_TO_MILLIMETERS = 25.4
        private const val FEET_TO_MILLIMETERS = 304.8
        private const val YARDS_TO_MILLIMETERS = 914.4

        private fun fromUnit(value: Double, scalar: Double) = Distance(value * scalar)

        /**
         * Creates a new [Distance] from a distance in millimeters
         * @param millimeters the distance in millimeters
         */
        @JvmStatic
        fun fromMm(millimeters: Double) = Distance(millimeters)

        /**
         * Creates a new [Distance] from a distance in millimeters
         * @param millimeters the distance in millimeters
         */
        @JvmStatic
        fun fromMm(millimeters: Int) = fromMm(millimeters.toDouble())

        /**
         * Creates a new [Distance] from a distance in centimeters
         * @param centimeters the distance in centimeters
         */
        @JvmStatic
        fun fromCm(centimeters: Double) = fromUnit(centimeters, CENTIMETERS_TO_MILLIMETERS)

        /**
         * Creates a new [Distance] from a distance in centimeters
         * @param centimeters the distance in centimeters
         */
        @JvmStatic
        fun fromCm(centimeters: Int) = fromCm(centimeters.toDouble())

        /**
         * Creates a new [Distance] from a distance in meters
         * @param meters the distance in meters
         */
        @JvmStatic
        fun fromMeters(meters: Double) = fromUnit(meters, METERS_TO_MILLIMETERS)

        /**
         * Creates a new [Distance] from a distance in meters
         * @param meters the distance in meters
         */
        @JvmStatic
        fun fromMeters(meters: Int) = fromMeters(meters.toDouble())

        /**
         * Creates a new [Distance] from a distance in inches
         * @param inches the distance in inches
         */
        @JvmStatic
        fun fromIn(inches: Double) = fromUnit(inches, INCHES_TO_MILLIMETERS)

        /**
         * Creates a new [Distance] from a distance in inches
         * @param inches the distance in inches
         */
        @JvmStatic
        fun fromIn(inches: Int) = fromIn(inches.toDouble())

        /**
         * Creates a new [Distance] from a distance in feet
         * @param feet the distance in feet
         */
        @JvmStatic
        fun fromFt(feet: Double) = fromUnit(feet, FEET_TO_MILLIMETERS)

        /**
         * Creates a new [Distance] from a distance in feet
         * @param feet the distance in feet
         */
        @JvmStatic
        fun fromFt(feet: Int) = fromFt(feet.toDouble())

        /**
         * Creates a new [Distance] from a distance in yards
         * @param yards the distance in yards
         */
        @JvmStatic
        fun fromYd(yards: Double) = fromUnit(yards, YARDS_TO_MILLIMETERS)

        /**
         * Creates a new [Distance] from a distance in yards
         * @param yards the distance in yards
         */
        @JvmStatic
        fun fromYd(yards: Int) = fromYd(yards.toDouble())
    }

    /**
     * The value of the distance in millimeters
     */
    @JvmField
    val inMm = value

    /**
     * The value of the distance in centimeters
     */
    @JvmField
    val inCm = value / CENTIMETERS_TO_MILLIMETERS

    /**
     * The value of the distance in meters
     */
    @JvmField
    val inMeters = value / METERS_TO_MILLIMETERS

    /**
     * The value of the distance in inches
     */
    @JvmField
    val inIn = value / INCHES_TO_MILLIMETERS

    /**
     * The value of the distance in feet
     */
    @JvmField
    val inFt = value / FEET_TO_MILLIMETERS

    /**
     * The value of the distance in yards
     */
    @JvmField
    val inYd = value / YARDS_TO_MILLIMETERS

    /**
     * Creates a new instance of [Distance] with the given value
     * @param value the value in millimeters to create an instance with
     */
    override fun newInstance(value: Double): Distance = Distance(value)

    override fun toString(): String = "$value mm"
}

/**
 * Creates a new [Distance] from a distance in millimeters
 */
val Double.mm: Distance get() = Distance.fromMm(this)

/**
 * Creates a new [Distance] from a distance in millimeters
 */
val Int.mm: Distance get() = Distance.fromMm(this)

/**
 * Creates a new [Distance] from a distance in centimeters
 */
val Double.cm: Distance get() = Distance.fromCm(this)

/**
 * Creates a new [Distance] from a distance in centimeters
 */
val Int.cm: Distance get() = Distance.fromCm(this)

/**
 * Creates a new [Distance] from a distance in meters
 */
val Double.m: Distance get() = Distance.fromMeters(this)

/**
 * Creates a new [Distance] from a distance in meters
 */
val Int.m: Distance get() = Distance.fromMeters(this)

/**
 * Creates a new [Distance] from a distance in inches
 */
val Double.inches: Distance get() = Distance.fromIn(this)

/**
 * Creates a new [Distance] from a distance in inches
 */
val Int.inches: Distance get() = Distance.fromIn(this)

/**
 * Creates a new [Distance] from a distance in inches
 */
val Double.inch: Distance get() = Distance.fromIn(this)

/**
 * Creates a new [Distance] from a distance in inches
 */
val Int.inch: Distance get() = Distance.fromIn(this)

/**
 * Creates a new [Distance] from a distance in feet
 */
val Double.ft: Distance get() = Distance.fromFt(this)

/**
 * Creates a new [Distance] from a distance in feet
 */
val Int.ft: Distance get() = Distance.fromFt(this)

/**
 * Creates a new [Distance] from a distance in yards
 */
val Double.yd: Distance get() = Distance.fromYd(this)

/**
 * Creates a new [Distance] from a distance in yards
 */
val Int.yd: Distance get() = Distance.fromYd(this)