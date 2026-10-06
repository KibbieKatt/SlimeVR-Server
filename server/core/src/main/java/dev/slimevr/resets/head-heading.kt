package dev.slimevr.resets

import io.github.axisangles.ktmath.Quaternion
import io.github.axisangles.ktmath.Vector3
import kotlin.math.atan2

// Small thrshold to treat ear-down orientation as overlapping planes
private const val MIN_INTERSECTION_LENGTH_SQUARED = 1e-6f

/**
 * Uses the intersection of the headset's forward/up plane with a
 * world-horizontal plane to determine where the headset gaze direction
 * intersects with the horizon when projected along the headset vertical plane.
 *
 * Headset right is the headset plane's normal, so world-up crossed with headset
 * right gives the directed horizontal heading (ie where that plane intersects
 * with the horizontal plane). This points along gaze when upright,
 * headset-down when looking up, and headset-up when looking down.
 *
 * When one ear is vertical, the planes coincide, so the original quaternion
 * yaw projection is used as it produces the correct result.
 */
internal fun headResetHeading(referenceRotation: Quaternion): Quaternion {
	// Get quaternion squared magnitude
	val lengthSquared = referenceRotation.lenSq()
	// Preserve original behavior if quaternion is not useful
	if (!lengthSquared.isFinite() || lengthSquared <= 0f) return referenceRotation

	// Get unit rotation
	val rotation = referenceRotation.unit()
	// World-up crossed with world-space headset-right gives the plane
	// intersection direction vector
	val heading = Vector3.POS_Y.cross(rotation.sandwichUnitX())
	// Use the intersection unless the planes nearly coincide
	if (heading.lenSq() > MIN_INTERSECTION_LENGTH_SQUARED) {
		// Convert vector to quaternion and pick the headset forwards direction
		return Quaternion.rotationAroundYAxis(atan2(-heading.x, -heading.z)).twinNearest(rotation)
	}

	// Cross product under threshold means near overlapping planes. This means
	// one ear is pointing directly up/down and gaze is near horizontal.
	// Passing through the original reference results in the gaze direction
	// being used.
	return referenceRotation
}
