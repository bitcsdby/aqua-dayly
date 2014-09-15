

#include <math.h>

#include <delay.h>
#include <packet.h>

#include <packet-stamp.h>
#include <antenna.h>
#include <mobilenode.h>
#include <propagation.h>
#include <wireless-phy.h>
#include "cubepropagation.h"


static class CubePropagationClass: public TclClass {
public:
        CubePropagationClass() : TclClass("Propagation/CubePropagation") {}
        TclObject* create(int, const char*const*) {
                return (new CubePropagation);
        }
} class_cubepropagation;

CubePropagation::CubePropagation()
{
  printf("cubepropagation: I am called return %f\n", TRANSMISSION_DISTANCE);
  last_hr = last_ht = 0.0;
  crossover_dist = 0.0;
}

   double
   ThreeRayGetDist(double Pr, double Pt, double Gt, double Gr, double ht, double hr)

{
     printf("cubepropagation: I am called return %f\n", TRANSMISSION_DISTANCE);
  double d=TRANSMISSION_DISTANCE;
   return d;
       /* Get quartic root */
       //return sqrt(sqrt(Pt * Gt * Gr * (hr * hr * ht * ht) / Pr));
}
/* End -NEW- */

// use Friis at less than crossover distance
// use two-ray at more than crossover distance
//static double
double CubePropagation::ThreeRay(double Pt, double Gt, double Gr, double ht, double hr, double L, double d)
{
        return Pt;
        /*
         *  Two-ray ground reflection model.
         *
         *	     Pt * Gt * Gr * (ht^2 * hr^2)
         *  Pr = ----------------------------
         *           d^4 * L
         *
         * The original equation in Rappaport's book assumes L = 1.
         * To be consistant with the free space equation, L is added here.
         */
	//  return Pt * Gt * Gr * (hr * hr * ht * ht) / (d * d * d * d * L);
}

double
CubePropagation::Pr(PacketStamp *t, PacketStamp *r, WirelessPhy *ifp)
{
printf("cubepropagation: I am called return %f\n", TRANSMISSION_DISTANCE);
	double Pr =t->getTxPr();
        return Pr; 
}
