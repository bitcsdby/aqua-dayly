
#ifndef ns_vectorbasedforward_h
#define ns_vectorbasedforward_h

#include <assert.h>
#include <math.h>
#include <stdio.h>
//#include <signal.h>
//#include <float.h>
#include <stdlib.h>

#include <tcl.h>

#include "agent.h"
#include "tclcl.h"
#include "config.h"
#include "packet.h"
#include "trace.h"
#include "random.h"
#include "classifier.h"
#include "underwatersensor/uw_common/underwatersensornode.h"
#include "arp.h"
#include "mac.h"
#include "ll.h"
#include "dsr/path.h"
#include "uw_routing_buffer.h"

#define THIS_NODE             here_
#define JITTER                0.08       // (sec) to jitter broadcast
#define DELAY                 0.2
//#define DELAY2                0.01

#define SEND_MESSAGE(x,y,z)  send_to_dmux(prepare_message(x,y,z), 0)

#define INTEREST      1
#define DATA          2
#define DATA_READY    3
#define SOURCE_DISCOVERY 4
#define SOURCE_TIMEOUT   5
#define TARGET_DISCOVERY 6
#define TARGET_REQUEST 7
#define SOURCE_DENY  8
#define V_SHIFT 9
#define FLOODING 10 
#define DATA_TERMINATION 11



#define MAX_ATTRIBUTE 3
#define MAX_NEIGHBORS 30
#define MAX_DATA_TYPE 30
#define MAX_NEIGHBOR 10
#define WINDOW_SIZE  19 

//used by hash table to limited the maximum length

//#define ROUTING_PORT 255


typedef struct Position{
  double x;
  double y;
  double z;
} position;

typedef struct RoutingVector{
    position start;
    position end;
} routing_vector;

typedef struct Neighbornode{
      routing_vector    vec;
     position node;   
}neighbornode;


typedef struct Neighborhood{
  int number;
  neighbornode  neighbor[MAX_NEIGHBOR];
} neighborhood;


struct uw_extra_info {

  // ns_addr_t osender_id;            // The original sender of this message
  // unsigned int seq;           //  sequence number

  double ox;  // the start point of the forward path
  double oy;
  double oz;

  //ns_addr_t sender_id;            // The forwarder of this message

  double fx;  // the forward 's position
  double fy;
  double fz;
 
// the end point of the forward path
  double tx; 
  double ty;
  double tz;

// this is the information about relative position of the receiver to the forwarder, not include in the header of real packet
  double dx;
  double dy;
  double dz; 

};


struct hdr_uwvb{
	unsigned int mess_type;
	unsigned int pk_num;
        ns_addr_t target_id; // the target id  of this data 
        ns_addr_t sender_id;  //original sender id
  // nsaddr_t next_nodes[MAX_NEIGHBORS];
  //int      num_next;
        unsigned int data_type;
        ns_addr_t forward_agent_id;// the forwarder id

       struct uw_extra_info info;
       position original_source;
    //  double token;
  	double ts_;                       // Timestamp when pkt is generated.
      double range;    // target range
	static int offset_;
  	inline static int& offset() { return offset_; }
  	inline static hdr_uwvb* access(const Packet*  p) {
		return (hdr_uwvb*) p->access(offset_);
	}
};





class UWPkt_Hash_Table {
 public:
  Tcl_HashTable htable;

  UWPkt_Hash_Table() {
    window_size=WINDOW_SIZE;
    Tcl_InitHashTable(&htable, 3);
  }

  int  window_size;
  void reset();
  void delete_hash(hdr_uwvb*); //delete the enrty that has the same key as the new packet
  void delete_hash(ns_addr_t, unsigned); 

  void put_in_hash(hdr_uwvb *);
  void put_in_hash(hdr_uwvb *, const position *, const position*, const position*);
  neighborhood* GetHash(ns_addr_t sender_id, unsigned int pkt_num);
};


class UWData_Hash_Table {
 public:
  Tcl_HashTable htable;

  UWData_Hash_Table() {
    Tcl_InitHashTable(&htable, MAX_ATTRIBUTE);
  }

  void reset();
  void PutInHash(int *attr);
  Tcl_HashEntry  *GetHash(int *attr);
};



class VectorbasedforwardAgent;

class UWDelayHandler: public Handler{
public:
UWDelayHandler(VectorbasedforwardAgent * a):Handler(){a_=a;}
void handle(Event* e);
protected :
  VectorbasedforwardAgent * a_;
  
};

class UWVoidAvoidHandler: public Handler{
public:
UWVoidAvoidHandler(VectorbasedforwardAgent * a):Handler(){a_=a;}
void handle(Event* e);
protected :
  VectorbasedforwardAgent * a_;
};

class UWFloodingHandler: public Handler{
public:
UWFloodingHandler(VectorbasedforwardAgent * a):Handler(){a_=a;}
 void handle(Event*);
protected :
  VectorbasedforwardAgent * a_;
};

class UWFloodingForwardHandler: public Handler{
public:
UWFloodingForwardHandler(VectorbasedforwardAgent * a):Handler(){a_=a;}
 void handle(Event*);
protected :
  VectorbasedforwardAgent * a_;
};

class VectorbasedforwardAgent : public Agent {
 public:
  VectorbasedforwardAgent();
  int command(int argc, const char*const* argv);
  void recv(Packet*, Handler*);

  // Vectorbasedforward_Entry routing_table[MAX_DATA_TYPE];

 protected:
  int pk_count;
  int counter;
  double priority;
  const double mini_distance;// distance used for flooding packet delay
  const double mini_threshold;// desirablenss used for normal data packet delay
  bool measureStatus;  //?? where do I use this?
  const  int control_packet_size;
 
  // int port_number;
  UWPkt_Hash_Table PktTable;
  UWPkt_Hash_Table SourceTable;
  UWPkt_Hash_Table Target_discoveryTable;
  UWPkt_Hash_Table SinkTable;
  UWPkt_Hash_Table CenterPktTable; 
  UWPkt_Hash_Table DataTerminationPktTable; 

  UWDelayHandler delayhandler; 
  UWVoidAvoidHandler void_avoidance_handler;
  UWFloodingHandler  flooding_handler;
  UWFloodingForwardHandler flooding_forward_handler;
  
  RoutingBuffer void_avoidance_buffer;
  RoutingBuffer receiving_buffer;
  
  UnderwaterSensorNode *node;
  Trace *tracetarget;       // Trace Target
  NsObject *ll;  
  NsObject *port_dmux;
  double width; 
// the width is used to test if the node is close enough to the path specified by the packet  
  

   inline void send_to_dmux(Packet *pkt, Handler *h) { 
    port_dmux->recv(pkt, h); 
  }

  void Terminate();
  void reset();
  void ConsiderNew(Packet*);

  void set_delaytimer(Packet*,double);
  //void set_shift_timer(ns_addr_t,int,double);
  void set_shift_timer(Packet*,double);
  // void set_flooding_timer(ns_addr_t,int,double);
//  void set_flooding_forward_timer(Packet*, double);

  void process_flooding_timeout(Packet*);
  void process_flooding_forward_timeout(Packet*);
  void process_void_avoidance_timeout(Packet*);
  void process_void_avoidance_timeout(ns_addr_t,position*,position*,int);

  void processFloodingPacket(Packet*);
  void timeout(Packet*);
 
  void makeCopy(Packet*);
  void sendFloodingPacket(Packet*);
  // void sendVectorShiftPacket(ns_addr_t, int);
  void sendDataTermination(const Packet*);

  double advance(Packet *);
  double distance(const Packet *);
  double projection(Packet*);
  double projection(const position*, const position*, const position *);
  double calculateDelay(Packet*, position*);
  double calculateDelay(const position*,const position*,const position*, const position*);
  double calculateFloodingDesirableness(const Packet*);
  double calculateDesirableness(const Packet*);

  Packet* generateVectorShiftPacket(const ns_addr_t*, int,const position*, const position*);
  void calculatePosition(Packet*);
  void setMeasureTimer(Packet*,double);

  bool IsVoidNode(const neighbornode*,const position*,const position*,const position*,int);
  bool IsTarget(Packet*);
  bool IsCloseEnough(Packet*);
  bool IsSamePosition(const position*, const position*);
  bool IsControlMessage(const Packet*);
 
//  Packet *create_packet();
//  Packet *prepare_message(unsigned int dtype, ns_addr_t to_addr, int msg_type);

  
  void DataForSink(Packet *pkt);
  // void StopSource();
  void MACprepare(Packet *pkt);
  void MACsend(Packet *pkt, Time delay=0);

  void trace(char *fmt,...);
  friend class UWDelayHandler;
  friend class UWVoidAvoidHandler;
  friend class UWFloodingHandler;
  friend class UWFloodingForwardHandler;
};



#endif




