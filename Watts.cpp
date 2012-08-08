// Convert LTC5507 DC measurement to Watts 

// Copyright (c) 2012 jeff millar, wa1hco
// Distributed under the GNU GPL v2. For full terms see the file COPYING at the GNU website.

//   Vmeas: output of LTC5507, minus Vol 
//   Vol: Vout with no RF, varies for each sensor 
//   dB_coupler: Attenuation of the direction coupler 
#include <math.h> 
float Watts( float dB_coupler, float Vol, float V ) 
{ 
  float Vinpp, Watts, Gain_coupler; 
  if (V < 0.0) { V = 0.0; } 
  if (V >= 0.42100) // if measurement above mid point 
  { // Use high curve fit
    Vinpp = 0.036838*pow(V,2) + 1.739364*V + -0.384399; 
  } 
  else
  { // Use low curve fit
    Vinpp = 7.346069*pow(V,3) + -3.656032*pow(V,2) + 1.032727*V + 0.017345;
  } 
  Gain_coupler = pow(10, -dB_coupler/10); 
  Watts = Gain_coupler * pow(Vinpp/2.818, 2)/50; // P = E^2/R
  return Watts; 
}  // Watts()
