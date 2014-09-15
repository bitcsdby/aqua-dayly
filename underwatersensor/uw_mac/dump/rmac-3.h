#ifndef ns_rmac_h
#define ns_rmac_h

#include "underwatermac.h"
#include "config.h"
#include "packet.h"


#define TABLE_SIZE 10 // the size of delay table
#define MAXIMUMBACKOFF 4 // the maximum times of backoffs
#define BACKOFF 1 //deleted later, used by TxProcess


#define UW_ND 1
#define UW_ACK_ND 2



struct time_record{
  int node_addr;// the address of the node
  double arrival_time;// the time to receive the ND packet from the node
  double sending_time; // the sending time of ND in local clock
};

struct latency_record{
  int node_addr;      // the address of the node
  double latency;    // the propagation latency with that node
  double sumLatency;// the sum of latency
  int num;         // number of ACKND packets 
  double last_update_time; // the time of last update
};



struct hdr_nd{
  // unsigned int type;     //packet type
        unsigned int pk_num;    // sequence number
        int sender_addr;  //original sender' address
  //	double ts;           // Timestamp when pkt is generated.
	
	static int offset_;
  	inline static int& offset() { return offset_; }
  	inline static hdr_nd* access(const Packet*  p) {
		return (hdr_nd*) p->access(offset_);
	}
};

struct hdr_ack_nd{
  //	unsigned int type;
        unsigned int pk_num;    // sequence number
        int sender_addr;  //original sender' address
         double ts;// sending time of the ND in sender's clock

       //  struct  time_record table[TABLE_SIZE]; // delay table
	
	static int offset_;
  	inline static int& offset() { return offset_; }

  	inline static hdr_ack_nd* access(const Packet*  p) {
		return (hdr_ack_nd*) p->access(offset_);
	}
};



class RMac;

class NDBackoffHandler: public Handler{
 public:
  NDBackoffHandler(RMac*);
  void handle(Event*);
  void clear();
 private:
  int counter_;
  RMac* mac_;
};


class NDHandler: public Handler{
 public:
  NDHandler(RMac*);
  void handle(Event*);
 private:
  RMac* mac_;
};


class NDStatusHandler: public Handler{
 public:
  NDStatusHandler(RMac*);
  void handle(Event*);
 private:
  RMac* mac_;
};


class AckNDWindowHandler: public Handler{
 public:
  AckNDWindowHandler(RMac*);
  void handle(Event*);
 private:
  RMac* mac_;
};

class PhaseOneHandler: public Handler{
 public:
  PhaseOneHandler(RMac*);
  void handle(Event*);
 private:
  RMac* mac_;
};


class RMac: public UnderwaterMac {
   
public:
        RMac();
     
       	int  command(int argc, const char*const* argv);
        double  ND_window_;// the window to send ND
        double  ACKND_window_;// the winddow to send ACK_ND
        double  PhaseOne_window_; // the time for latency deytection
        int PhyOverhead_;// the overhead caused by phy layer
        int arrival_table_index;
        int latency_table_index;
        int num_send; 
        int packet_size_;
        
        int PhaseOne_cycle_; // number of cycles in phase one
        
	double cycle_start_time; // the begining time of this cycle;  
        
         struct  time_record arrival_table[TABLE_SIZE]; 
         struct latency_record latency_table[TABLE_SIZE];            
  



 void InitPhaseOne(double/*ND window*/,double/*ack_nd window*/,double/* phaseOne window*/); 
    
     void InitND(double/*ND window*/,double/*ack_nd window*/,double/* phase One window*/);// to detect latency 
     void SendND();
     void TxND(Packet*);
     // void ProcessPacket(Packet*);   
     void ProcessNDPacket(Packet*);   
     void ProcessACKNDPacket(Packet*);   

     void SendAckND();
     void StatusProcess(Event*);
    
         Event nd_event;
         Event status_event;
         Event acknd_event;         
         Event phaseone_event;

         NDStatusHandler status_handler;
         NDHandler nd_handler;
         NDBackoffHandler backoff_handler;
         AckNDWindowHandler acknd_window_handler;         
         PhaseOneHandler phaseone_handler;

        
              

        //Node* node(void) const {return node_;}
        // to process the incomming packet
        virtual  void RecvProcess(Packet*);
      
       // to process the outgoing packet
        virtual  void TxProcess(Packet*);

protected:        
	inline int initialized() {
	return  UnderwaterMac::initialized();
	}
 private:
	//	double interval_ND_ACKND;
        friend class NDBackoffHandler;
        friend class AckNDWindowHanlder;
        friend class NDHandler;
        friend class Status_handler;
        friend class PhaseOne_handler;
};

#endif /* __rmac_h__ */

