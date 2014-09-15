
#ifndef __cubepropagation_h__
#define __cubepropagation_h__

#define TRANSMISSION_DISTANCE 20 //transmission distance is 20 meters


#include <packet-stamp.h>
#include <wireless-phy.h>
#include <propagation.h>

/* -NEW- */
/*double
FrissGetDist(double Pr, double Pt, double Gt, double Gr, double lambda,
            double L);
*/
double
ThreeRayGetDist(double Pr, double Pt, double Gt, double Gr, double ht,
              double hr);
/* End -NEW- */

class CubePropagation : public Propagation {
public:
  CubePropagation();
  virtual double Pr(PacketStamp *tx, PacketStamp *rx, WirelessPhy *ifp);

protected:
  double ThreeRay(double Pt, double Gt, double Gr, double ht, double hr, double L, double d);
  double last_hr, last_ht;
  double crossover_dist;
};


#endif /* __cubepropagation_h__ */
