# Drop-off Detection

`Firmware/lib/sensors/src/dropoff.c`. Geometric reasoning over the
*absence* of returns: a step down or hole in the floor shows up as
missing pixels in the bottom rows of the ToF grid where we expect floor
hits.

## Logic

For each pixel that looks at least ~11.5° below horizontal in world
frame, the module knows the world-frame ray (from the same
pre-computed table the obstacle pipeline uses) and the predicted
distance at which that ray should intersect the floor plane
`Z = −1.20 m`.

The pipeline applies a +45° optical-axis roll to the VL53L5CX rays
before this module sees them. The detector is **rotation-invariant** —
it uses each ray's world-frame Z component, not the pixel's grid
position — so the eligible "floor rays" are now the lower triangle of
the diamond (corner pixel pointing straight down, plus its neighbours).
The corner pixel strikes the floor at ~1.89 m forward instead of the
unrotated bottom-row's 2.31 m; the detection envelope is therefore
slightly closer-in but compensated by the wider lateral spread of
floor rays across the diamond's bottom half.

A pixel votes for drop-off if either:

1. it returned **no valid status** while a floor return was expected, **or**
2. its measured range exceeds **1.3×** the predicted floor range — i.e.
   the floor stepped down at least far enough to push the hit out of the
   tolerance.

If **≥ 3 pixels** in the expected-floor band vote, `HAPNAV_OBS_FLAG_DROPOFF`
is set on the frame.

## Why a square-wave on the wristband

A drop-off is qualitatively different from a wall: walls warrant
"slow down", drop-offs warrant "stop now". To make that distinction
unambiguous on the haptic side, the wristband translates the flag into a
distinctive square-wave pattern on **all four** LRAs in unison (150 ms
on / 150 ms off, ≈3.3 Hz). See
[`07_haptic_policy.md`](07_haptic_policy.md).

Pulse-on-all-four is a deliberately rare cue in normal proximity output
— normal output decorrelates across channels by direction — so the
"everything pulsing" pattern reliably reads as "stop".

## Tuning

Threshold (`≥ 3 pixels` and `1.3× ratio`) lives in `dropoff.c`. Tightening
either reduces false drop-off cues at the cost of late detection; loosening
raises false positives, which the user perceives as a phantom cliff —
much worse than a missed wall, since they will physically stop walking.
