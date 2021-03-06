/*
 * Copyright (C) 2014 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma version(1)
#pragma rs java_package_name(com.kunato.imagestitching)

rs_allocation gCurrentFrame;

uchar4 __attribute__((kernel)) convertFrames(uint32_t x, uint32_t y) {

    // Read in pixel values from latest frame - YUV color space

    uchar4 curPixel;
    curPixel.r = rsGetElementAtYuv_uchar_Y(gCurrentFrame, x, y);
    curPixel.g = rsGetElementAtYuv_uchar_U(gCurrentFrame, x, y);
    curPixel.b = rsGetElementAtYuv_uchar_V(gCurrentFrame, x, y);
    curPixel.a = 255;
    int4 rgb;
    rgb.r = curPixel.r +
            curPixel.b * 1436 / 1024 - 179;
    rgb.g = curPixel.r -
            curPixel.g * 46549 / 131072 + 44 -
            curPixel.b * 93604 / 131072 + 91;
    rgb.b = curPixel.r +
            curPixel.g * 1814 / 1024 - 227;
    rgb.a = 255;


    // Write out merged HDR result
    uchar4 out = convert_uchar4(clamp(rgb, 0, 255));

    return out;
}
