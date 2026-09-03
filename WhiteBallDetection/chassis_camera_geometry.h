#ifndef CHASSIS_CAMERA_GEOMETRY_H
#define CHASSIS_CAMERA_GEOMETRY_H

/* Physical camera-to-chassis calibration inherited from the four-channel
 * Camera_Lazy virtual-IR strip: total width 20%, shifted 7% left from the
 * raw camera centre. The middle channels span 38%..48%, with the chassis
 * forward axis at raw x=43%.
 *
 * Detection and TFT output are rotated by 180 degrees, so the corresponding
 * corrected-frame positions are 52%..62%, with the axis at 57%. */
#define CHASSIS_RAW_CENTER_LEFT_FRACTION 0.38f
#define CHASSIS_RAW_PUSH_AXIS_FRACTION 0.43f
#define CHASSIS_RAW_CENTER_RIGHT_FRACTION 0.48f

#define CHASSIS_CORRECTED_CENTER_LEFT_FRACTION \
    (1.0f - CHASSIS_RAW_CENTER_RIGHT_FRACTION)
#define CHASSIS_CORRECTED_PUSH_AXIS_FRACTION \
    (1.0f - CHASSIS_RAW_PUSH_AXIS_FRACTION)
#define CHASSIS_CORRECTED_CENTER_RIGHT_FRACTION \
    (1.0f - CHASSIS_RAW_CENTER_LEFT_FRACTION)

/* Detector centre_x uses -1 at the left and +1 at the right. */
#define CHASSIS_CENTER_BAND_LEFT_X \
    (2.0f * CHASSIS_CORRECTED_CENTER_LEFT_FRACTION - 1.0f)
#define CHASSIS_PUSH_AXIS_CENTER_X \
    (2.0f * CHASSIS_CORRECTED_PUSH_AXIS_FRACTION - 1.0f)
#define CHASSIS_CENTER_BAND_RIGHT_X \
    (2.0f * CHASSIS_CORRECTED_CENTER_RIGHT_FRACTION - 1.0f)

#endif
