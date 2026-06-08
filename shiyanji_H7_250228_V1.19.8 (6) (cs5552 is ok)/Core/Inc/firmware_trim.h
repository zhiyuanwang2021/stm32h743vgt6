#ifndef FIRMWARE_TRIM_H
#define FIRMWARE_TRIM_H

/* Release-size trim switches: keep protocol/control logic unchanged and
 * only mute verbose debug formatting that bloats FLASH.
 */
#define FW_TRIM_DEBUG_PRINTF          1
#define FW_TRIM_COMM_INFO_LOGS        1
#define FW_TRIM_CONTROL_INFO_LOGS     1
#define FW_TRIM_INOUT_INFO_LOGS       1
#define FW_TRIM_POSGEN_FLOAT_LOGS     1

#endif /* FIRMWARE_TRIM_H */
