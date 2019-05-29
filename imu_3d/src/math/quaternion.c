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

#include <math.h>
#include "quaternion.h"

void q_quatFromUserFrame(struct Quaternion* quat,
                         const struct freespace_UserFrame* userFrame) {
    quat->w = userFrame->angularPosA;
    quat->x = userFrame->angularPosB;
    quat->y = userFrame->angularPosC;
    quat->z = userFrame->angularPosD;

    q_normalize(quat, quat);
}

void q_conjugate(struct Quaternion* out,
                 const struct Quaternion* q) {
    out->w = q->w;
    out->x = -q->x;
    out->y = -q->y;
    out->z = -q->z;
}

float q_length(const struct Quaternion* q) {
    return sqrtf(q_lengthSq(q));
}

float q_lengthSq(const struct Quaternion* q) {
    return ((q->w * q->w) + (q->x * q->x) + (q->y * q->y) + (q->z * q->z));
}

void q_scale(struct Quaternion* out,
             const struct Quaternion* q,
             float scale) {
    out->w = q->w * scale;
    out->x = q->x * scale;
    out->y = q->y * scale;
    out->z = q->z * scale;
}

void q_normalize(struct Quaternion* out, const struct Quaternion* q) {
    float len = q_length(q);
    q_scale(out, q, 1.0f / len);
}

void q_toEulerAngles(struct Vec3f* out, const struct Quaternion* q) {
    float m11 = (2.0f * q->w * q->w) + (2.0f * q->x * q->x) - 1.0f;
    float m12 = (2.0f * q->x * q->y) + (2.0f * q->w * q->z);
    float m13 = (2.0f * q->x * q->z) - (2.0f * q->w * q->y);
    float m23 = (2.0f * q->y * q->z) + (2.0f * q->w * q->x);
    float m33 = (2.0f * q->w * q->w) + (2.0f * q->z * q->z) - 1.0f;

    out->x = atan2f(m23, m33);
    out->y = asinf(-m13);
    out->z = atan2f(m12, m11);
}
