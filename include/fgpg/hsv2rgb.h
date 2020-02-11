#pragma once

#include<cmath> // Needed for fmod()

/*
 * H(Hue): 0 - 360 degree (integer)
 * S(Saturation): 0 - 1.00 (double)
 * V(Value): 0 - 1.00 (double)
 * 
 * output[3]: Output, array size 3, int
 */
void HSVtoRGB(int H, double S, double V, int output[3]);