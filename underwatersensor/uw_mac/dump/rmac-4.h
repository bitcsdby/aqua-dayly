#ifndef ns_rmac_h
#define ns_rmac_h

#include "underwatermac.h"
#include "config.h"
#include "packet.h"


#define TABLE_SIZE 10 // the size of delay table
#define MAXIMUMBACKOFF 4 // the maximum times of backoffs
#define BACKOFF 1 //deleted later, used by TxProcess
#define MAXIMUM_BUFFER 4 


#define UW_ND 1
#define UW_ACK_ND 2


#define PHASEONE 1
#define PHASETWO 2
#define PHASETHREE 3


struct buffer_cell{
  Packet* packet;
  buffer_cell * next;
};


class TransmissionBuffer{
 public: 
       TransmissionBuffer(){
                  head_=NULL; 
                  num_of_packet=0;
                          };
      
    void AddNewPacket(Packet*);
    int  DeletePacket(Packet*);
    bool IsEmpty();
    bool IsFull();
    buffer_cell * lookup(Packet*);
    int num_of_packet;// number of sending packets
 private:
       buffer_cell* head_;
};



struct time_record{
  int node_addr;// the address of the node
  double arrival_time;// the time to receive the ND packet from the node
  double sending_time; // the sending time of ND in local clock
};

struct period_record{
  int node_addr;// the address of the node
  double difference;// the difference with my period
  double duration; // duration of duty cycle
  double last_update_time; // the time last updated
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
  double arrival_time; //arrival time of ND in  the receiver's clock
       //  struct  time_record table[TABLE_SIZE]; // delay table
	
	static int offset_;
  	inline static int& offset() { return offset_; }

  	inline static hdr_ack_nd* access(const Packet*  p) {
		return (hdr_ack_nd*) p->access(offset_);
	}
};

struct hdr_syn{
        //  unsigned int type;     //packet type
        unsigned int pk_num;    // sequence number
        int sender_addr;  //original sender' address
        double interval;    // interval to the begining of periodic operation
        double duration; // duration of duty cycle;

	static int offset_;
  	inline static int& offset() { return offset_; }
  	inline static hdr_syn* access(const Packet*  p) {
		return (hdr_syn*) p->access(offset_);
	}
};





class RMac;

class NDBackoffHandler: public Handler{
 public:
  NDBackoffHandler(RMac*);
  void handle(Event*);
  void clear();
  double window_;

 private:
  int counter_;
  RMac* mac_;
};


class LargeNDHandler: public Handler{
 public:
  LargeNDHandler(RMac*);
  void handle(Event*);
 private:
  RMac* mac_;
};

class ShortNDHandler: public Handler{
 public:
  ShortNDHandler(RMac*);
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


class LargeAckNDWindowHandler: public Handler{
 public:
  LargeAckNDWindowHandler(RMac*);
  void handle(Event*);
 private:
  RMac* mac_;
};

class ShortAckNDWindowHandler: public Handler{
 public:
  ShortAckNDWindowHandler(RMac*);
  void handle(Event*);
 private:
  RMac* mac_;
};


class ACKNDHandler: public Handler{
 public:
  ACKNDHandler(RMac*);
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

class PhaseTwoHandler: public Handler{
 public:
  PhaseTwoHandler(RMac*);
  void handle(Event*);
 private:
  RMac* mac_;
};

class PhaseThreeHandler: public Handler{
 public:
  PhaseThreeHandler(RMac*);
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
        double  PhaseOne_window_; // the time for latency detection
        double  PhaseTwo_window_; // the time for SYN announcement
    

        int PhyOverhead_;// the overhead caused by phy layer
        int arrival_table_index;
        int large_latency_table_index;
        int short_latency_table_index;
        int period_table_index;
        int num_send; 
        int large_packet_size_;
        int short_packet_size_;   
        double duration_; // duration of duty cycle     
        double IntervalPhase2Phase3_;
        double next_period;//the start_time of next duty cycle


        int PhaseOne_cycle_; // number of cycles in phase one
        int PhaseStatus;         

	double cycle_start_time; // the begining time of this cycle;  
        
         struct  time_record arrival_table[TABLE_SIZE]; 
         struct latency_record large_latency_table[TABLE_SIZE];            
         struct latency_record short_latency_table[TABLE_SIZE];    
         struct period_record  period_table[TABLE_SIZE];
         TransmissionBuffer txbuffer;

 void InitPhaseOne(double/*ND window*/,double/*ack_nd window*/,double/* phaseOne window*/); 
    
 void InitPhaseTwo(); 
 void InitPhaseThree();

     void InitND(double/*ND window*/,double/*ack_nd window*/,double/* phase One window*/);// to detect latency 

     void SendND(int);
     void TxND(Packet*, double);
     // void ProcessPacket(Packet*);   
     void ProcessNDPacket(Packet*);   
     void ProcessLargeACKNDPacket(Packet*);   
     void ProcessShortACKNDPacket(Packet*);   
     void ProcessSYN(Packet*);

     void SendLargeAckND();
     void SendShortAckND();
     void StatusProcess(Event*);
     void SendSYN();
    

         Event large_nd_event;
         Event short_nd_event;
         Event status_event;
         Event large_acknd_event;
         Event short_acknd_event;         
         Event phaseone_event;
         Event phasetwo_event;
         Event phasethree_event;

         NDStatusHandler status_handler;
         LargeNDHandler large_nd_handler;
         ShortNDHandler short_nd_handler;
         NDBackoffHandler backoff_handler;
         
         ACKNDHandler acknd_handler;
         LargeAckNDWindowHandler large_acknd_window_handler;         
         ShortAckNDWindowHandler short_acknd_window_handler; 

         PhaseOneHandler phaseone_handler;
         PhaseTwoHandler phasetwo_handler; 
         PhaseThreeHandler phasethree_handler;
        
              
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
        friend class PhaseOneHandler;
        friend class PhaseTwoHandler;
        friend class PhaseThreeHandler;
};

#endif /* __rmac_h__ */

