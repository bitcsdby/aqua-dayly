
#include "packet.h"
#include "random.h"
#include "underwatersensor/uw_common/underwatersensornode.h"
#include "mac.h"
#include "rmac.h"
#include "underwaterphy.h"
#include "random.h"



int hdr_nd::offset_;
int hdr_ack_nd::offset_;


static class RMAC_ND_HeaderClass: public PacketHeaderClass{
 public:
 RMAC_ND_HeaderClass():PacketHeaderClass("Packetheader/RMAC_ND",sizeof(hdr_nd))
{
 bind_offset(&hdr_nd::offset_);
}
} class_rmac_nd_hdr;


static class RMAC_ACKND_HeaderClass: public PacketHeaderClass{
 public:
  RMAC_ACKND_HeaderClass():PacketHeaderClass("Packetheader/RMAC_ACKND",sizeof(hdr_ack_nd))
{
 bind_offset(&hdr_ack_nd::offset_);
}
} class_rmac_ack_nd_hdr;




NDBackoffHandler::NDBackoffHandler(RMac* p):mac_(p),counter_(0){}
 
void NDBackoffHandler::handle(Event*e)
{
  counter_++;
  if(counter_<MAXIMUMBACKOFF)
    mac_->TxND((Packet*)e);
  else 
    {
    clear();
  printf("Rmac:backoff:too many backoffs\n");
  Packet::free((Packet*)e);
    }
}

void NDBackoffHandler::clear(){
counter_=0;
}

NDStatusHandler::NDStatusHandler(RMac* p):mac_(p){}
void NDStatusHandler::handle(Event* e)
{
  mac_->StatusProcess(e);
}





NDHandler::NDHandler(RMac* p):mac_(p){}

void NDHandler::handle(Event* e)
{ 
    mac_->SendND();
}


ACKNDHandler::ACKNDHandler(RMac* p):mac_(p){}

void ACKNDHandler::handle(Event* e)
{ 
    mac_->TxND((Packet*) e);
}





AckNDWindowHandler::AckNDWindowHandler(RMac* p):mac_(p){}

void AckNDWindowHandler::handle(Event* e)
{ 
    mac_->SendAckND();
}



/* ======================================================================
    RMAC for  underwater sensor
   ====================================================================== */
static class RMacClass : public TclClass {
public:
 RMacClass():TclClass("Mac/UnderwaterMac/RMac") {}
   TclObject* create(int, const char*const*) {
	  return (new RMac());
   }
} class_rmac;


RMac::RMac() :UnderwaterMac(),backoff_handler(this),nd_handler(this),acknd_handler(this),acknd_window_handler(this), status_handler(this)
{
  num_send=0;
  arrival_table_index=0;
  latency_table_index=0;
 
  for(int i=0;i<TABLE_SIZE;i++){
    arrival_table[i].node_addr=-1;
    latency_table[i].node_addr=-1;
    latency_table[i].num=0;
    latency_table[i].last_update_time=0.0;
  }

  packet_size_=30;// deleted later used in TxProcess
  
  bind("Latency_window_",&Latency_window_);
  bind("ND_window_",&ND_window_); 
  bind("ACKND_window_",&ACKND_window_); 
  bind("PhyOverhead_",&PhyOverhead_);

  //interval_ND_ACKND=Latency_window_-ND_window_-ACKND_window_;
  InitND(ND_window_,ACKND_window_, Latency_window_);
}


void 
RMac::InitND(double t1,double t2, double t3)
{
  double delay=Random::uniform()*t1;
  Scheduler& s=Scheduler::instance();
  double delay2=(t3-t2)+Random::uniform()*t2;
 
  s.schedule(&nd_handler, &nd_event, delay);

  s.schedule(&acknd_window_handler,&acknd_event,delay2);
  return;
}


void 
RMac::SendND()
{
      Packet* pkt =Packet:: alloc();
      hdr_nd* ndh = HDR_ND(pkt);
      // hdr_ip* iph = HDR_IP(pkt);
      hdr_cmn*  cmh = HDR_CMN(pkt);
      
   
  // additional 2*8 denotes the size of type,next-hop of the packet and 
  // timestamp
      cmh->size()=sizeof(hdr_nd)+3*8;

       cmh->next_hop()=MAC_BROADCAST;
       cmh->direction()=hdr_cmn::DOWN; 
       cmh->addr_type()=NS_AF_ILINK;
       cmh->ptype_=PT_ND;
      
       
      // ndh->type = UW_ND;
      ndh->pk_num = num_send;
      // ndh->ts=NOW;
      ndh->sender_addr= node_->address();

      num_send++;
      // lastND_time=NOW;

      // iph->src_.addr_=node_->address();
      // iph->dst_.addr_=node_->address();
      //iph->dst_.port_=255;     

 printf("rmac SendND:node(%d) send ND  packet  %d at %f\n", ndh->sender_addr,ndh->pk_num, NOW);
      TxND(pkt);  
}


void 
RMac::SendAckND()
{
  if (arrival_table_index==0) return;// not ND received


  while(arrival_table_index>0){ 
      Packet* pkt = Packet::alloc();
      hdr_ack_nd* ackndh = HDR_ACK_ND(pkt);
    
      hdr_cmn*  cmh = HDR_CMN(pkt);
      
      ackndh->pk_num = num_send;
      ackndh->sender_addr=node_->address();
      num_send++;

      cmh->ptype_=PT_ACK_ND;
        
         int index1=-1;
        index1=rand()%arrival_table_index; 
        double t2=-0.1;
        double t1=-0.1;
    
        int receiver=arrival_table[index1].node_addr;
         t2=arrival_table[index1].arrival_time; 
         t1=arrival_table[index1].sending_time; 

	 for(int i=index1;i<arrival_table_index;i++){
	   arrival_table[i].node_addr=arrival_table[i+1].node_addr;
           arrival_table[i].sending_time=arrival_table[i+1].sending_time;
           arrival_table[i].arrival_time=arrival_table[i+1].arrival_time;
	 }

   
          ackndh->arrival_time=t2;
          ackndh->ts_=t1;
 // additional 2*8 denotes the size of type,next-hop of the packet and 
 // timestamp
      cmh->size()=sizeof(hdr_ack_nd)+3*8;

      cmh->next_hop()=receiver;
      cmh->direction()=hdr_cmn::DOWN; 
      cmh->addr_type()=NS_AF_ILINK;

      
         Scheduler& s=Scheduler::instance();
         double delay=Random::uniform()*ACKND_window_;
         s.schedule(&acknd_handler, (Event*) pkt, delay);

	 arrival_table_index--;
  }


          return; 
}





void 
RMac::TxND(Packet* pkt)
{
 
  hdr_cmn* cmh=HDR_CMN(pkt);
 
  assert(initialized());
  UnderwaterSensorNode* n=(UnderwaterSensorNode*) node_;

 
  hdr_cmn::access(pkt)->txtime()=(cmh->size()*encoding_efficiency_+
                                   PhyOverhead_)/bit_rate_;

  double txtime=hdr_cmn::access(pkt)->txtime();

  if(SLEEP==n->TransmissionStatus()) {
  Poweron();
  n->SetTransmissionStatus(SEND);
  cmh->ts_=NOW;
  
  sendDown(pkt);
  backoff_handler.clear();

 
  Scheduler& s=Scheduler::instance();
  s.schedule(&status_handler,&status_event,txtime);
  return;
  }

  if(IDLE==n->TransmissionStatus()){
  
  n->SetTransmissionStatus(SEND);
 
  // printf("TxND the data type is %d\n",MAC_BROADCAST);
  //  printf("broadcast : I am going to send the packet down tx is %f\n",txtime);
   cmh->ts_=NOW;
  sendDown(pkt);
  backoff_handler.clear();
  //  printf("broadcast %d Tx Idle set timer at %f tx is %f\n",node_->nodeid(),NOW,txtime);
  Scheduler& s=Scheduler::instance();
  s.schedule(&status_handler,&status_event,txtime);
  return;
  }

  if(RECV==n->TransmissionStatus())
    {
      Scheduler& s=Scheduler::instance();
      double d1=ND_window_-NOW;
 
      if(d1>0){
      double backoff=Random::uniform()*d1;
      
   // printf("broadcast Tx set timer at %f backoff is %f\n",NOW,backoff);
      s.schedule(&backoff_handler,(Event*) pkt,backoff);
      return;
      }
      else {
          backoff_handler.clear();
          printf("Rmac:backoff:no time left \n");
          Packet::free(pkt);
      }

    }

if (SEND==n->TransmissionStatus())
{
  // this case is supposed not to  happen 
    printf("rmac: queue send data too fas\n");
    Packet::free(pkt);
      return;
}

}



void 
RMac::ProcessNDPacket(Packet* pkt)
{
    hdr_nd* ndh=HDR_ND(pkt);
    hdr_cmn* cmh=HDR_CMN(pkt);

    int  sender=ndh->sender_addr;
    double time=NOW;
    if(arrival_table_index>=TABLE_SIZE){ 
      printf("rmac:ProcessNDPacket:arrival_table is full\n");
      return;
    }
    arrival_table[arrival_table_index].node_addr=sender;
    arrival_table[arrival_table_index].arrival_time=time;
    arrival_table[arrival_table_index].sending_time=cmh->ts_;
    
    arrival_table_index++;
    Packet::free(pkt);
    return;
}


void 
RMac::ProcessACKNDPacket(Packet* pkt)
{
    hdr_ack_nd* ackndh=HDR_ACK_ND(pkt);
    hdr_cmn* cmh=HDR_CMN(pkt);

    int  sender=ackndh->sender_addr;
    double t4=NOW;
    double t3=cmh->ts_;
    int myaddr=node_->address();
 
    double t2=ackndh->arrival_time;
    double t1=ackndh->ts_;

double latency=((t4-t1)-(t3-t2))/2.0;
bool newone=true;   
   
 for (int i=0;i<TABLE_SIZE;i++)
 if (latency_table[i].node_addr==sender)
      {
       latency_table[i].sumLatency+=latency;
       latency_table[i].num++;
       latency_table[i].last_update_time=NOW;
       latency_table[i].latency = 
                  latency_table[i].sumLatency/latency_table[i].num;
       newone=false;
      }
 
 if(newone)
{

    if(latency_table_index>=TABLE_SIZE){ 
      printf("rmac:ProcessNDPacket:arrival_table is full\n");
      return;
    }

    latency_table[latency_table_index].node_addr=sender;
    latency_table[latency_table_index].sumLatency+=latency;
    latency_table[latency_table_index].num++;
    latency_table[latency_table_index].last_update_time=NOW;
    latency_table[latency_table_index].latency = 
          latency_table[latency_table_index].sumLatency/latency_table[latency_table_index].num;
    latency_table_index++;
}
 for(int i=0;i<latency_table_index;i++)
   printf("node (%d) to node (%d) latency is %f\n", myaddr, latency_table[i].node_addr, latency_table[i].latency); 




 Packet::free(pkt);
 return;

}



/*
 this program is used to handle the received packet, 
it should be virtual function, different class may have 
different versions.
*/


void 
RMac::RecvProcess(Packet* pkt){
   hdr_cmn* cmh=HDR_CMN(pkt);
   int dst=cmh->next_hop();
 
  printf("rmac:node %d  gets a packet at time %f \n",node_->address(),NOW);
  if(dst==MAC_BROADCAST){
    
    if (cmh->ptype_==PT_ND) ProcessNDPacket(pkt); //this is ND packet      
    return;
  }

   if(dst==index_){
      
       printf("rmac:this is my ACKND packet \n");
     if (cmh->ptype_==PT_ACK_ND) ProcessACKNDPacket(pkt); 
     
     // uptarget_->recv(pkt, this);
    return;
}
    printf("underwaterbroadcastmac: this is neither broadcast nor my packet, just drop it\n");
   Packet::free(pkt);
   return;
}

/*
 this program is used to handle the transmitted packet, 
it should be virtual function, different class may have 
different versions.
*/


void 
RMac::TxProcess(Packet* pkt){
  

  // printf("broadcast receive packet at  %f \n",NOW);
  hdr_cmn* cmh=HDR_CMN(pkt);
  assert(initialized());
  UnderwaterSensorNode* n=(UnderwaterSensorNode*) node_;

   cmh->size()=packet_size_;
   hdr_cmn::access(pkt)->txtime()=(cmh->size()*encoding_efficiency_)/bit_rate_;
   double txtime=hdr_cmn::access(pkt)->txtime();

  if(SLEEP==n->TransmissionStatus()) {
  Poweron();
  n->SetTransmissionStatus(SEND);

  cmh->next_hop()=MAC_BROADCAST;
  cmh->direction()=hdr_cmn::DOWN; 
  cmh->addr_type()=NS_AF_ILINK;
  sendDown(pkt);
  backoff_handler.clear();

  // printf("broadcast Tx sleep set timer at %f tx is %f\n",NOW,txtime);
  Scheduler& s=Scheduler::instance();
  s.schedule(&status_handler,&status_event,txtime);
  return;
  }

  if(IDLE==n->TransmissionStatus()){
  
  n->SetTransmissionStatus(SEND);
  cmh->next_hop()=MAC_BROADCAST;
  cmh->direction()=hdr_cmn::DOWN; 
  cmh->addr_type()=NS_AF_ILINK;
  //  printf("broadcast : I am going to send the packet down tx is %f\n",txtime);
  sendDown(pkt);
 backoff_handler.clear();
  //  printf("broadcast %d Tx Idle set timer at %f tx is %f\n",node_->nodeid(),NOW,txtime);
  Scheduler& s=Scheduler::instance();
  s.schedule(&status_handler,&status_event,txtime);
  return;
  }

  if(RECV==n->TransmissionStatus())
    {
      Scheduler& s=Scheduler::instance();
      double backoff=Random::uniform()*BACKOFF;

      //   printf("broadcast Tx set timer at %f backoff is %f\n",NOW,backoff);
      s.schedule(&backoff_handler,(Event*) pkt,backoff);
      return;
    }

if (SEND==n->TransmissionStatus())
{
  // this case is supposed not to  happen 
   printf("broadcast: queue send data too fas\n");
    Packet::free(pkt);
      return;
}
}
   
void 
RMac::StatusProcess(Event* p)
{
// printf("broadcast status process1\n"); 
  if(callback_) callback_->handle(&status_event);

 // printf("broadcast status process2\n"); 
 UnderwaterSensorNode* n=(UnderwaterSensorNode*) node_;
 // printf("broadcast status process3\n"); 
 if(SLEEP==n->TransmissionStatus()) return;

   n->SetTransmissionStatus(IDLE);
   // printf("broadcast exit in statusprocess\n"); 
 return;
}




int
RMac::command(int argc, const char*const* argv)
{


     if(argc == 3) {
		TclObject *obj;
                 if (strcmp(argv[1], "node_on") == 0) {
		   Node* n1=(Node*) TclObject::lookup(argv[2]);
		   if (!n1) return TCL_ERROR;
		   node_ =n1; 
		   return TCL_OK;
		 }
     }

	return UnderwaterMac::command(argc, argv);
}


