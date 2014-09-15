#ifndef ns_rmac_h
#define ns_rmac_h

#include "underwatermac.h"
#include "config.h"
#include "packet.h"


#define TABLE_SIZE 8 // the size of delay table
#define MAXIMUMBACKOFF 4 // the maximum times of backoffs
#define BACKOFF 1 //deleted later, used by TxProcess


#define UW_ND 1
#define UW_ACK_ND 2



struct time_record{
  int node_addr;// the address of the node
  double sending_time; //sending time of ND in receiver 'clock
  double arrival_time;// the time to receive the ND packet from the node
};

struct latency_record{
  int node_addr;      // the address of the node
  double latency;    // the propagation latency with that node
  double sumLatency;// the sum of latency
  int num;         // number of ACKND packets 
  double last_update_time; // the time of last update
};



struct hdr_nd{
  // unsigned int type;          //packet type
        unsigned int pk_num;    // sequence number
        int sender_addr;       //original sender' address
        int receiver_addr;     //receiver's address
   	double ts_;           // not used
        double arrival_time;      //not used
	
	static int offset_;
  	inline static int& offset() { return offset_; }
  	inline static hdr_nd* access(const Packet*  p) {
		return (hdr_nd*) p->access(offset_);
	}
};


struct hdr_ack_nd{
  // unsigned int type;          //packet type
        unsigned int pk_num;    // sequence number
        int sender_addr;       //original sender' address
        int receiver_addr;     //receiver's address
        double ts_;               //sending time of ND in receiver 'clock
   	double arrival_time;   //arrival time of ND from the receiver;  
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

class ACKNDHandler: public Handler{
 public:
  ACKNDHandler(RMac*);
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




class RMac: public UnderwaterMac {
   
public:
        RMac();
     
       	int  command(int argc, const char*const* argv);
        double  ND_window_;// the window to send ND
        double  ACKND_window_;// the winddow to send ACK_ND
        double  Latency_window_; // the time for latency deytection
        int PhyOverhead_;// the overhead caused by phy layer
        int arrival_table_index;
        int latency_table_index;
        int num_send; 
        int packet_size_;  //deleted later, used in Txprocess 
        //double lastND_time; // the time to send the last ND packet;  

     //  arrival time of ND packet for each  neighbor 
         struct  time_record arrival_table[TABLE_SIZE]; 
         struct latency_record latency_table[TABLE_SIZE];            
  





     void InitND(double/*ND window*/,double/*ack_nd window*/,double/* latency window*/);// to detect latency 
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

         NDStatusHandler status_handler;
         NDHandler nd_handler;
         ACKNDHandler acknd_handler;
         NDBackoffHandler backoff_handler;
         AckNDWindowHandler acknd_window_handler;         


        
              

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
        friend class ACKNDHandler;
        friend class Status_handler;
};

#endif /* __rmac_h__ */

