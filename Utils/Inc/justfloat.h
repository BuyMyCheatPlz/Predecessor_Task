/* justfloat minimal header */
#ifndef JUSTFLOAT_H
#define JUSTFLOAT_H

#include <stdint.h>

/* Convert float to string with given decimal precision.
 * buf should be large enough to hold result.
 * Example: jf_ftoa(-12.34f, buf, 2) -> "-12.34"
 */
void jf_ftoa(float value, char *buf, int precision);

#endif /* JUSTFLOAT_H */
