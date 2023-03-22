#pragma once

#include<cmath> // Needed for fmod()

/*
 * H(Hue): 0 - 360 degree (integer)
 * S(Saturation): 0 - 1.00 (float)
 * V(Value): 0 - 1.00 (float)
 * 
 * output[3]: Output, array size 3, int
 */
void HSVtoRGB(int H, float S, float V, int output[3]);