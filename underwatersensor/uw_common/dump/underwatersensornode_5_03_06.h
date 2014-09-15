

#ifndef __ns_underwatersensornode_h__
#define __ns_underwatersensornode_h__

#include "object.h"
#include "trace.h"
#include "lib/bsd-list.h"
#include "phy.h"
#include "topography.h"
#include "arp.h"
#include "node.h"
#include "gridkeeper.h" // I am not sure if it is useful in our case
#include "energy-model.h"
#include "location.h"

class GridKeeper; // think it later

 enum TransmissionStatus {SLEEP,IDLE,SEND,RECV};



#if COMMENT_ONLY
		 -----------------------
		|			|
		|	Upper Layers	|
		|			|
		 -----------------------
		    |		    |
		    |		    |
		 -------	 -------
		|	|	|	|
		|  LL	|	|  LL	|
		|	|	|	|
		 -------	 -------
		    |		    |
		    |		    |
		 -------	 -------
		|	|	|	|
		| Queue	|	| Queue	|
		|	|	|	|
		 -------	 -------
		    |		    |
		    |		    |
		 -------	 -------
		|	|	|	|
		|  Mac	|	|  Mac	|
		|	|	|	|
		 -------	 -------
		    |		    |
		    |		    |
		 -------	 -------	 -----------------------
		|	|	|	|	|			|
		| Netif	| <---	| Netif | <---	|	Mobile Node	|
		|	|	|	|	|			|
		 -------	 -------	 -----------------------
		    |		    |
		    |		    |
		 -----------------------
		|			|
		|	Channel(s) 	|
		|			|
		 -----------------------
#endif
class UnderwaterPositionHandler : public Handler {
public:
	UnderwaterPositionHandler(UnderwaterSensorNode* n) : node(n) {}
	void handle(Event*);
private:
	UnderwaterSensorNode *node;
};
	





	 
class UnderwaterSensorNode : public MobileNode 
{
 	friend class UnderwaterPositionHandler;
public:
     
	UnderwaterSensorNode();
	virtual int command(int argc, const char*const* argv);
         double propdelay(UnderwaterSensorNode*);
       	void	move();
	void start();
           int clearSinkStatus();// added by peng xie
           int setSinkStatus();// added by peng xie
	 
         // added by peng Xie 
      inline double CX() { return CX_; }
      inline double CY() { return CY_; }
      inline double CZ() { return CZ_; }
     inline int sinkStatus(){return sinkStatus_;}
     inline int failure_status(){return failure_status_;}
     inline double failure_pro(){return failure_pro_;}
     inline void SetTransmissionStatus(enum TransmissionStatus i){trans_status=i;}
     inline enum TransmissionStatus TransmissionStatus(){return trans_status;}
         double CX_;
         double CY_;
         double CZ_;
	 int sinkStatus_;
         int failure_status_;
      
	enum  TransmissionStatus trans_status;
//add by peng xie, 1  indicates the node is in failure state, 0 normal state

         double failure_pro_;
 // add by peng xie to indicate the error probability of sending packets
	//   int destination_status; 

protected:

        double max_speed;
        double min_speed;
        void random_speed();

private:
	void		random_position();
	void		bound_position();
	int		random_motion_;	// is mobile

        Topography* T_;

};

#endif // ns_underwatersensornode_h






