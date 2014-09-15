#include <math.h>
#include <packet.h>
#include <phy.h>
#include "underwaterpropagation.h"
#include "underwatersensor/uw_common/underwatersensornode.h"
#include <modulation.h>
//#include <omni-antenna.h>
#include  "underwaterphy.h"
#include <packet.h>
//#include <ip.h>
//#include <agent.h>
//#include <trace.h>


//#include "diffusion/diff_header.h"

//#define max(a,b) (((a)<(b))?(b):(a))

void Underwater_Idle_Timer::expire(Event *) {
	a_->UpdateIdleEnergy();
}

/*
void UnderwaterPhy_Status_Timer::handle(Event* )
{
  Schedule& s=Scheduler::instance();
  phy->ResetStatus();
}
*/

/* ======================================================================
   UnderwaterPhy Interface
   ====================================================================== */
static class UnderwaterPhyClass: public TclClass {
public:
        UnderwaterPhyClass() : TclClass("Phy/UnderwaterPhy") {}
        TclObject* create(int, const char*const*) {
                return (new UnderwaterPhy);
        }
} class_UnderwaterPhy;

 
UnderwaterPhy::UnderwaterPhy() : Phy(), idle_timer_(this)
{
	/*
	 *  It sounds like 10db should be the capture threshold.
	 *
	 *  If a node is presently receiving a packet a a power level
	 *  Pa, and a packet at power level Pb arrives, the following
	 *  comparion must be made to determine whether or not capture
	 *  occurs:
	 *
	 *    10 * log(Pa) - 10 * log(Pb) > 10db
	 *
	 *  OR equivalently
	 *
	 *    Pa/Pb > 10.
	 *
	 */

 
	bind("CPThresh_", &CPThresh_);
	bind("CSThresh_", &CSThresh_);
	bind("RXThresh_", &RXThresh_);
	//bind("bandwidth_", &bandt* lookup(scheduler_uid_t uid) = 0;	// look for eventt* lookup(scheduler_uid_t uid) = 0;	// look for eventwidth_);
	bind("Pt_", &Pt_);
	bind("freq_", &freq_);
	bind("L_", &L_);
	bind("K_", &K_);

	node_ = 0;
	propagation_ = 0;
	modulation_ = 0;

	// Assume AT&T's Wavelan PCMCIA card -- Chalermek
        //	Pt_ = 8.5872e-4; // For 40m transmission range.
	//      Pt_ = 7.214e-3;  // For 100m transmission range.
	//      Pt_ = 0.2818; // For 250m transmission range.
	//	Pt_ = pow(10, 2.45) * 1e-3;         // 24.5 dbm, ~ 281.8mw
	
	Pt_consume_ = 0.660;  // 1.6 W drained power for transmission
	Pr_consume_ = 0.395;  // 1.2 W drained power for reception

	//	P_idle_ = 0.035; // 1.15 W drained power for idle

	P_idle_ = 0.0;

	//channel_idle_time_ = NOW;
	update_energy_time_ = NOW;
	//last_send_time_ = NOW;
	
	idle_timer_.resched(1.0);
}

int
UnderwaterPhy::command(int argc, const char*const* argv)
{
	TclObject *obj; 

	if (argc==2) {
	        if (strcasecmp(argv[1], "PowerOn") == 0) {
			if (em() == NULL) 
				return TCL_OK;
			power_on();
			return TCL_OK;
		}
        	else if (strcasecmp(argv[1], "PowerOff") == 0) {
			if (em() == NULL) 
			   return TCL_OK;
			power_off();
			return TCL_OK;
		}

	} else if(argc == 3) {
		if (strcasecmp(argv[1], "setTxPower") == 0) {
			Pt_consume_ = atof(argv[2]);
	  //   printf("wireless-phy.cc: Pt_consume_=%f\n",Pt_consume_);
			return TCL_OK;
		} else if (strcasecmp(argv[1], "setRxPower") == 0) {
			Pr_consume_ = atof(argv[2]);
			return TCL_OK;
		} else if (strcasecmp(argv[1], "setIdlePower") == 0) {
			P_idle_ = atof(argv[2]);
			return TCL_OK;
		} else if( (obj = TclObject::lookup(argv[2])) == 0) {
			fprintf(stderr,"UnderwaterPhy: %s lookup of %s failed\n", 
				argv[1], argv[2]);
			return TCL_ERROR;
		} else if (strcmp(argv[1], "propagation") == 0) {
			assert(propagation_ == 0);
       fprintf(stderr,"UnderwaterPhy: there is propagation!!\n"); 
			propagation_ = (Propagation*) obj;
			return TCL_OK;
		} else if (strcasecmp(argv[1], "node") == 0) {
			assert(node_ == 0);
			node_ = (Node *)obj;
			return TCL_OK;
		}
	}
	return Phy::command(argc,argv);
}
 
void 
UnderwaterPhy::sendDown(Packet *p)
{
  UnderwaterSensorNode* n1;
  // printf("ok, this is underwaterphy senddown\n");

   
	/*
	 * Sanity Check
	 */

	assert(initialized());
	
	n1=(UnderwaterSensorNode*) node_;
        TransmissionStatus status=n1->TransmissionStatus(); 
       
	if (em()) 
		if ((status==SLEEP) || (em()->energy()<=0)) {
			Packet::free(p);
			return;
		}
		else {
		  // node is power-on and energy is greater 
          switch (status){

      case IDLE:
	{
		double txtime = hdr_cmn::access(p)->txtime();

		//set the status timer

		//         Scheduler & s=Scheduler::instance();
		// s.schedule(&statu_timer,&status_update,txtime);
		// status=SEND;
              
		    double start_time = NOW;
		    double end_time = NOW+txtime;

		    // update energy
		  if (start_time > update_energy_time_) {
			em()->DecrIdleEnergy(start_time - 
					   update_energy_time_, P_idle_);
			update_energy_time_ = start_time;
		    }
		  else printf("underwater phy: it is weired, perhaps it is caused by overlappd transmission\n");

	           em()->DecrTxEnergy(txtime, P_idle_);
		    update_energy_time_ =end_time;
		    break;
	}
	  case SEND:

	    /* something wrong in this case since the transmitter is 
               transmitting some packet while a new packet arrives. 
               In this simulator, we can't handle this case
	    */
                 printf("UnderwaterPhy:overlapped transmissions\n");
               
		 // Packet::free(p);
		 break;
	  case RECV:
	    {
	    // status_=SEND;
           	double txtime = hdr_cmn::access(p)->txtime();
                double end_time=NOW+txtime;
		/*  The receiver is receiving a packet when the 
                    transmitter begins to transmit a data. 
                    We assume the half-duplex mode, the transmitter 
                    stops the receiving process and begins the sending
                    process.
		 */

	             //set status timer
		// Scheduler & s=Scheduler::instance();
		//s.cancel(&status_update);
		// s.schedule(&statu_timer,&status_update,txtime);
		// status=SEND;

		if(update_energy_time_<end_time)
                         {
                 double  overlap_time=update_energy_time_-NOW;
                 double  actual_txtime=end_time-update_energy_time_;
                 em()->DecrTxEnergy(overlap_time,
					      Pt_consume_-Pr_consume_);
		 em()->DecrTxEnergy(actual_txtime,Pt_consume_); 
                 update_energy_time_=end_time;
			 }
		else {
 
                double overlap_time=txtime;
                  em()->DecrTxEnergy(overlap_time,
					      Pt_consume_-Pr_consume_);
		}
		break;
	    }
          SLEEP: printf("underwaterphy: I am sleeping!!\n");
		break;
	  default: printf("underwaterphy: no such default at all\n");
	  }//end of switch 

		}//end of else

	/*
           else {
			Packet::free(p);
			return;
	}
	*/
						     
	/*
	 *  Stamp the packet with the interface arguments
	 */

	p->txinfo_.stamp((MobileNode*)node(), ant_->copy(), Pt_, lambda_);

	// Send the packet
           printf("UnderwaterPhy:copy the packet to cgannel\n");
	channel_->recv(p, this);
}




int 
UnderwaterPhy::sendUp(Packet *p)
{
	/*
	 * Sanity Check
	 */

         UnderwaterSensorNode* n1;


        printf("underwater phy: I got the going-up packet\n");
	assert(initialized());

        	
	n1=(UnderwaterSensorNode*) node_;
        TransmissionStatus status=n1->TransmissionStatus(); 

	PacketStamp s;
	double Pr;
	int pkt_recvd = 0;
	
	// if the node is in sleeping mode, drop the packet simply

    if (em()) 
      if ((status==SEND || status==SLEEP)||(em()->energy()<=0)) 
               {
			return pkt_recvd;
	       }
       
	if(propagation_) {
		s.stamp((MobileNode*)node(), ant_, 0, lambda_);
		Pr = propagation_->Pr(&p->txinfo_, &s, this);
		if (Pr < CSThresh_) {
	       printf("underwaterphy: Pr<CSThresh_,signal is too weak\n");
			return pkt_recvd;		       
		}
	
	}

	if(modulation_) {
		hdr_cmn *hdr = HDR_CMN(p);
		hdr->error() = modulation_->BitError(Pr);
	}
	

        double rcvtime = hdr_cmn::access(p)->txtime();

         //set status timer
        //  Scheduler & s=Scheduler::instance();
       //       if (status_==RECV) s.cancel(&status_update);
       //              s.schedule(&statu_timer,&status_update,txtime);
        //              	status_=RECV;

	p->txinfo_.RxPr = Pr;
	p->txinfo_.CPThresh = CPThresh_;
        
         double start_time =NOW;
	 double end_time = NOW+rcvtime;
	 
	if (start_time > update_energy_time_) {
	     em()->DecrIdleEnergy(start_time-update_energy_time_,P_idle_);
	     em()->DecrRcvEnergy(rcvtime,Pr_consume_);
	     update_energy_time_ = end_time;
	}
	else{
          /* In this case, this node is receiving some packet*/ 
	  if(end_time>update_energy_time_){
             em()->DecrRcvEnergy(end_time-update_energy_time_,Pr_consume_);
	     update_energy_time_ = end_time;
	  }
	}
 
	if (Pr < RXThresh_) {

			/*
			 * We can detect, but not successfully receive
			 * this packet.
			 */

			return pkt_recvd;		     		      
	}

		if (em()->energy() <= 0) {  
			// saying node died
			em()->setenergy(0);
			((MobileNode*)node())->log_energy(0);
		}
                
		return 1;
}


void
UnderwaterPhy::power_on()
{
        if (em() == NULL)
 	    return;	
   	if (NOW > update_energy_time_) {
      	    update_energy_time_ = NOW;
   	}
}

void 
UnderwaterPhy::power_off()
{
	if (em() == NULL)
            return;
        if (NOW > update_energy_time_) {
            em()->DecrIdleEnergy(NOW-update_energy_time_,
                                P_idle_);
            update_energy_time_ = NOW;
	}

	/*
	//shut off the status_timer
      Scheduler & s=Scheduler::instance();
      s.cancel(&status_update);
                  status=SLEEP;
	*/
}

void
UnderwaterPhy::dump(void) const
{
	Phy::dump();
	fprintf(stdout,
			"\tPt: %f, Gt: %f, Gr: %f, lambda: %f, L: %f\n",
	Pt_, ant_->getTxGain(0,0,0,lambda_), ant_->getRxGain(0,0,0,lambda_), lambda_, L_);
	//fprintf(stdout, "\tbandwidth: %f\n", bandwidth_);
	fprintf(stdout, "--------------------------------------------------\n");
}


void UnderwaterPhy::UpdateIdleEnergy()
{
	if (em() == NULL) {
		return;
	}
	if (NOW > update_energy_time_ && em()->node_on()) {
		  em()-> DecrIdleEnergy(NOW-update_energy_time_,
					P_idle_);
		  update_energy_time_ = NOW;
	}

	// log node energy
	if (em()->energy() > 0) {
		((MobileNode *)node_)->log_energy(1);
        } else {
		((MobileNode *)node_)->log_energy(0);   
        }

	idle_timer_.resched(1.0);
}


/*
void
UnderwaterPhy::ResetStatus(){

	if (em() == NULL) {
		return;
	}
	if (em()->node_on())
	  status=IDLE
	    else status=SLEEP;
}
*/
