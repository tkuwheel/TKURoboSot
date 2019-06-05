/**
 * This file is part of libfreespace-examples.
 *
 * Copyright (c) 2009-2012, Hillcrest Laboratories, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of the Hillcrest Laboratories, Inc. nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef FREESPACE_QUATERNION_H_
#define FREESPACE_QUATERNION_H_

#include <freespace/freespace.h>
#include <freespace/freespace_codecs.h>
#include "vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

struct Quaternion {
    float x;
    float y;
    float z;
    float w;
};

/**
 * Convert Freespace UserFrame message to a normalized quaternion.
 *
 * Note: The returned quaternion gives the rotation in terms of
 * rotating the world around the object. It is usually desirable to
 * take the conjugate to rotate the object in a fixed reference frame.
 *
 * See game3d_example for usage.
 *
 * @param out the normalized quaternion is returned here
 * @param user a decoded user frame message
 */
void q_quatFromUserFrame(struct Quaternion* out,
                         const struct freespace_UserFrame* user);

/**
 * Put the conjugate of q in out
 */
void q_conjugate(struct Quaternion* out,
                 const struct Quaternion* q);

/**
 * return the length of q
 */
float q_length(const struct Quaternion* q);

/**
 * Return the length squared of q
 */
float q_lengthSq(const struct Quaternion* q);

/**
 * Scale q by scale and put the result in out
 */
void q_scale(struct Quaternion* out,
             const struct Quaternion* q,
             float scale);

/**
 * Normalize q and put the resulting quaternion in out
 */
void q_normalize(struct Quaternion* out,
                 const struct Quaternion* q);

/**
 * Do an euler decomposition of quaternion q for an aerospace sequence
 * rotation (yaw => pitch => roll).  In the resulting vector, x is
 * roll, y is pitch, z is yaw.
 */
void q_toEulerAngles(struct Vec3f* out,
                     const struct Quaternion* q);


#ifdef __cplusplus
}
#endif

#endif /* FREESPACE_QUATERNION_H_ */
