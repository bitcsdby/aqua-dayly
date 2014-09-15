#include "packet.h"
#include "random.h"
#include "underwatersensor/uw_common/underwatersensornode.h"
#include "mac.h"
#include "rmac.h"
#include "underwaterphy.h"
#include "random.h"



int hdr_nd::offset_;
int hdr_ack_nd::offset_;
int hdr_syn::offset_;



void 
TransmissionBuffer::AddNewPacket(Packet* p){
  buffer_cell* t2;
  buffer_cell* t1=new buffer_cell;

  t1->packet=p;
  t1->next=NULL;
  // insert this packet at the head of the link
  t2=head_;
  head_=t1;
  t1->next=t2;
  
  num_of_packet++;
  //printf("TransmissionBuffer: number of packet is %d\n",num_of_active_incomming_packet);
}


int 
TransmissionBuffer::DeletePacket(Packet* p){
  buffer_cell* t1;
  buffer_cell* t2;
  
  // insert this packet at the head of the link
  t2=head_;

  if (!t2) return 0;//0 no such point, 1:delete this point

  if (p==t2->packet){
    //    printf("underwatermac: the packet is at the head of list\n");
    head_=t2->next;
    num_of_packet--;
    delete t2;
    return 1;
}
  
  int modified=0;
  while(t2->next){
    if ((t2->next)->packet!=p) t2=t2->next;
    else{
    
     t1=t2->next;
     t2->next=t1->next;
     num_of_packet--;
    delete t1;
    modified=1;
    }
  }
  
  return modified;
}


buffer_cell*  
TransmissionBuffer::lookup(Packet* p){
  buffer_cell* t2;
  t2=head_;  
  while((t2->packet!=p)&&(!t2)) t2=t2->next;
  return t2;
}


bool 
TransmissionBuffer::IsEmpty(){
  return(0==num_of_packet);
}


bool 
TransmissionBuffer::IsFull(){
  return(MAXIMUM_BUFFER==num_of_packet);
}


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


static class RMAC_SYN_HeaderClass: public PacketHeaderClass{
 public:
 RMAC_SYN_HeaderClass():PacketHeaderClass("Packetheader/RMAC_SYN",sizeof(hdr_syn))
{
 bind_offset(&hdr_syn::offset_);
}
} class_rmac_syn_hdr;




NDBackoffHandler::NDBackoffHandler(RMac* p):mac_(p),window_(0),counter_(0){}
 
void NDBackoffHandler::handle(Event*e)
{
  counter_++;
  if(counter_<MAXIMUMBACKOFF)
    mac_->TxND((Packet*)e, window_);
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


LargeNDHandler::LargeNDHandler(RMac* p):mac_(p){}

void LargeNDHandler::handle(Event* e)
{ 
  /*
    mac_->arrival_table_index=0; 
  for(int i=0;i<TABLE_SIZE;i++)
    mac_->arrival_table[i].node_addr=-1;
  */
     mac_->cycle_start_time=NOW;
    mac_->SendND(mac_->large_packet_size_);
}


ShortNDHandler::ShortNDHandler(RMac* p):mac_(p){}

void ShortNDHandler::handle(Event* e)
{  
  /*
        mac_->arrival_table_index=0; 
  for(int i=0;i<TABLE_SIZE;i++)
    mac_->arrival_table[i].node_addr=-1;
  */
    mac_->cycle_start_time=NOW;
    mac_->SendND(mac_->short_packet_size_);
}



LargeAckNDWindowHandler::LargeAckNDWindowHandler(RMac* p):mac_(p){}

void LargeAckNDWindowHandler::handle(Event* e)
{ 
    mac_->SendLargeAckND();
}


ShortAckNDWindowHandler::ShortAckNDWindowHandler(RMac* p):mac_(p){}

void ShortAckNDWindowHandler::handle(Event* e)
{ 
   mac_->SendShortAckND();
}


ACKNDHandler::ACKNDHandler(RMac* p):mac_(p){}

void ACKNDHandler::handle(Event* e)
{ 
    mac_->TxND((Packet*) e, mac_->ACKND_window_);
}




PhaseOneHandler::PhaseOneHandler(RMac* p):mac_(p){}

void PhaseOneHandler::handle(Event* e)
{ 
    mac_->InitPhaseOne(mac_->ND_window_,mac_->ACKND_window_, mac_->PhaseOne_window_);
}



PhaseTwoHandler::PhaseTwoHandler(RMac* p):mac_(p){}

void PhaseTwoHandler::handle(Event* e)
{ 
    mac_->SendSYN();
}

PhaseThreeHandler::PhaseThreeHandler(RMac* p):mac_(p){}

void PhaseThreeHandler::handle(Event* e)
{ 
    mac_->InitPhaseThree();
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


RMac::RMac() :UnderwaterMac(),backoff_handler(this),large_nd_handler(this),short_nd_handler(this),large_acknd_window_handler(this),short_acknd_window_handler(this), status_handler(this), phaseone_handler(this),acknd_handler(this), phasetwo_handler(this), phasethree_handler(this)
{
  num_send=0;
  large_packet_size_=30;
  short_packet_size_=10;
  large_latency_table_index=0;
  short_latency_table_index=0;
  next_period=0;
  period_table_index=0;
  
 for(int i=0;i<TABLE_SIZE;i++){
    large_latency_table[i].node_addr=-1;
    large_latency_table[i].num=0;
    large_latency_table[i].last_update_time=0.0;

    short_latency_table[i].node_addr=-1;
    short_latency_table[i].num=0;
    short_latency_table[i].last_update_time=0.0;

    period_table[i].node_addr=-1;
    period_table[i].difference=0.0;
    period_table[i].last_update_time=0.0; 
  }

    arrival_table_index=0; 
  for(int i=0;i<TABLE_SIZE;i++)
    arrival_table[i].node_addr=-1;

  bind("PhaseOne_window_",&PhaseOne_window_);
  bind("PhaseTwo_window_",&PhaseTwo_window_);
  bind("duration_",&duration_);
  bind("ND_window_",&ND_window_); 
  bind("ACKND_window_",&ACKND_window_); 
  bind("PhyOverhead_",&PhyOverhead_);
  bind("large_packet_size_",&large_packet_size_);
  bind("short_packet_size_",&short_packet_size_);
  bind("PhaseOne_cycle_",&PhaseOne_cycle_);
  bind("IntervalPhase2Phase3_",&IntervalPhase2Phase3_);

  //  printf("size of table is %d, packet size is %d\n", sizeof(arrival_table),packet_size_); 
  //interval_ND_ACKND=Latency_window_-ND_window_-ACKND_window_;

  InitPhaseOne(ND_window_,ACKND_window_, PhaseOne_window_);
}

void 
RMac::InitPhaseOne(double t1,double t2, double t3)
{
 
   printf("RMac: Phaseone cycle: %d...\n",PhaseOne_cycle_);

   if(PhaseOne_cycle_)
    { 
      PhaseStatus=PHASEONE;
    Scheduler& s=Scheduler::instance();
    InitND(t1,t2,t3);
    s.schedule(&phaseone_handler,&phaseone_event,t3*2);
    PhaseOne_cycle_--;
    return;
    }

   InitPhaseTwo();
   return;
}

/*
void 
RMac::ProcessShortACKNDPacket(Packet* pkt)
{
    hdr_ack_nd* ackndh=HDR_ACK_ND(pkt);
    hdr_cmn* cmh=HDR_CMN(pkt);

    int  sender=ackndh->sender_addr;
    double t4=NOW;
    double t3=cmh->ts_;
    int myaddr=node_->address();
 
    double t2=ackndh->arrival_time;
    double t1=ackndh->ts;

double latency=((t4-t1)-(t3-t2))/2.0;
bool newone=true;   

Packet::free(pkt);
   
 for (int i=0;i<TABLE_SIZE;i++)
 if (short_latency_table[i].node_addr==sender)
      {
       short_latency_table[i].sumLatency+=latency;
       short_latency_table[i].num++;
       short_latency_table[i].last_update_time=NOW;
       short_latency_table[i].latency = 
                  short_latency_table[i].sumLatency/short_latency_table[i].num;
       newone=false;
      }
 
 if(newone)
{

    if(short_latency_table_index>=TABLE_SIZE){ 
      printf("rmac:ProcessNDPacket:arrival_table is full\n");
      return;
    }

    short_latency_table[short_latency_table_index].node_addr=sender;
    short_latency_table[short_latency_table_index].sumLatency+=latency;
    short_latency_table[short_latency_table_index].num++;
    short_latency_table[short_latency_table_index].last_update_time=NOW;
    short_latency_table[short_latency_table_index].latency = 
          short_latency_table[short_latency_table_index].sumLatency/short_latency_table[short_latency_table_index].num;
    short_latency_table_index++;
}
 for(int i=0;i<short_latency_table_index;i++)
   printf("node (%d) to node (%d) short latency is %f and number is %d\n", myaddr, short_latency_table[i].node_addr, short_latency_table[i].latency,short_latency_table[i].num); 
 
 return;

}

*/



void 
RMac::InitPhaseThree(){

  printf("RMac: this is InitPhaseThree\n"); 
    return;
}



void 
RMac::InitPhaseTwo(){

   double delay=Random::uniform()*PhaseTwo_window_;
   PhaseStatus=PHASETWO;

    cycle_start_time=NOW;
    next_period=IntervalPhase2Phase3_+PhaseTwo_window_+delay;
 
    printf("rmac initphasetwo: the phasethree of node %d is scheduled at %f\n",index_,NOW+next_period);
    Scheduler& s=Scheduler::instance();
    s.schedule(&phasetwo_handler, &phasetwo_event,delay);
    return;
}

void 
RMac::SendSYN(){

       Packet* pkt =Packet::alloc();
       hdr_syn* synh = HDR_SYN(pkt); 
       hdr_cmn*  cmh = HDR_CMN(pkt);
      
       cmh->size()=short_packet_size_;
       cmh->next_hop()=MAC_BROADCAST;
       cmh->direction()=hdr_cmn::DOWN; 
       cmh->addr_type()=NS_AF_ILINK;
       cmh->ptype_=PT_SYN;
      
      
       synh->pk_num = num_send;
       synh->sender_addr= node_->address();
     
       synh->duration=duration_;
        num_send++;

	printf("rmac SendSYN:node(%d) send SYN packet at %f\n", synh->sender_addr,NOW);
      TxND(pkt, PhaseTwo_window_);  
}


void 
RMac::InitND(double t1,double t2, double t3)
{

  
  // cycle_start_time=NOW;
  double delay=Random::uniform()*t1;
  double itval=(t3-t2-t1)/2.0;
  double delay2=(t1+itval)+Random::uniform()*t2;
  double delay3=(t3+t1+itval);

 
    Scheduler& s=Scheduler::instance();
   s.schedule(&large_nd_handler, &large_nd_event, delay);
   s.schedule(&large_acknd_window_handler,&large_acknd_event,delay2);

   s.schedule(&short_nd_handler, &short_nd_event, (t3+delay));
   s.schedule(&short_acknd_window_handler,&short_acknd_event,delay3);
  return;
}


void 
RMac::SendND(int pkt_size)
{
      Packet* pkt =Packet:: alloc();
      hdr_nd* ndh = HDR_ND(pkt);
      // hdr_ip* iph = HDR_IP(pkt);
      hdr_cmn*  cmh = HDR_CMN(pkt);
      
   
     // additional 2*8 denotes the size of type,next-hop of the packet and 
     // timestamp
  
       cmh->size()=sizeof(hdr_nd)+3*8;
      //  printf("old size is %d\n",cmh->size());
        cmh->size()=pkt_size;

       cmh->next_hop()=MAC_BROADCAST;
       cmh->direction()=hdr_cmn::DOWN; 
       cmh->addr_type()=NS_AF_ILINK;
       cmh->ptype_=PT_ND;
      
       
      // ndh->type = UW_ND;
      ndh->pk_num = num_send;
      // ndh->ts=NOW;
      ndh->sender_addr= node_->address();

      num_send++;
      //      lastND_time=NOW;

      // iph->src_.addr_=node_->address();
      // iph->dst_.addr_=node_->address();
      //iph->dst_.port_=255;     

 printf("rmac SendND:node(%d) send ND  packet  %d at %f\n", ndh->sender_addr,ndh->pk_num, NOW);
      TxND(pkt, ND_window_);  
}


void 
RMac::SendLargeAckND()
{
      Packet* pkt = Packet::alloc(sizeof(arrival_table));
      hdr_ack_nd* ackndh = HDR_ACK_ND(pkt);
    
      // initialize the arrival table


      hdr_cmn*  cmh = HDR_CMN(pkt);
      
    // ackndh->type = UW_ACK_ND;
       ackndh->pk_num = num_send;
      // ackndh->ts=NOW;
      ackndh->sender_addr=node_->address();
      num_send++;

      cmh->ptype_=PT_ACK_ND;

 // additional 2*8 denotes the size of type,next-hop of the packet and 
 // timestamp
      //cmh->size()=sizeof(hdr_ack_nd)+3*8;
       // printf("old size is %d\n",cmh->size());

       cmh->size()=large_packet_size_;


      cmh->next_hop()=MAC_BROADCAST;
      cmh->direction()=hdr_cmn::DOWN; 
      cmh->addr_type()=NS_AF_ILINK;

     

      memcpy(pkt->accessdata(),arrival_table,sizeof(arrival_table));

      // iph->src_.addr_=node_->address();
      // iph->dst_.addr_=node_->address();
      // iph->dst_.port_=255;     


 

  printf("rmac SendND:node(%d) send Large ACKND  at %f\n", ackndh->sender_addr, NOW);

  /*
    for(int i=0;i<TABLE_SIZE;i++){
       printf("node %d and arrival time %f\n", arrival_table[i].node_addr,arrival_table[i].arrival_time);	
     }
  */
       arrival_table_index=0; 
  for(int i=0;i<TABLE_SIZE;i++)
   arrival_table[i].node_addr=-1;
  
  

      TxND(pkt,ACKND_window_);  
}


void 
RMac::SendShortAckND()
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
          ackndh->ts=t1;
  // additional 2*8 denotes the size of type,next-hop of the packet and 
  // timestamp
    //  cmh->size()=sizeof(hdr_ack_nd)+3*8;
    

      cmh->size()=short_packet_size_;
      cmh->next_hop()=receiver;
      cmh->direction()=hdr_cmn::DOWN; 
      cmh->addr_type()=NS_AF_ILINK;

      
         Scheduler& s=Scheduler::instance();
         double delay=Random::uniform()*ACKND_window_;
         s.schedule(&acknd_handler, (Event*) pkt, delay);

	 arrival_table_index--;
  }


     arrival_table_index=0; 
  for(int i=0;i<TABLE_SIZE;i++)
    arrival_table[i].node_addr=-1;

          return; 
}


void 
RMac::TxND(Packet* pkt, double window)
{
 
  hdr_cmn* cmh=HDR_CMN(pkt);
  hdr_syn* synh = HDR_SYN(pkt); 
  
  assert(initialized());
  UnderwaterSensorNode* n=(UnderwaterSensorNode*) node_;

 
  hdr_cmn::access(pkt)->txtime()=(cmh->size()*encoding_efficiency_+
                                   PhyOverhead_)/bit_rate_;

  double txtime=hdr_cmn::access(pkt)->txtime();

  if(SLEEP==n->TransmissionStatus()) {
  Poweron();
  n->SetTransmissionStatus(SEND);

  
  cmh->ts_=NOW;

  if(PhaseStatus==PHASETWO){

    double t=NOW-cycle_start_time;

    synh->interval=next_period-t; 

    Scheduler & s=Scheduler::instance();
    s.schedule(&phasethree_handler,&phasethree_event,synh->interval);
  }

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

  if(PhaseStatus==PHASETWO){

   double t=NOW-cycle_start_time;
   synh->interval=next_period-t; 

    Scheduler & s=Scheduler::instance();
    s.schedule(&phasethree_handler,&phasethree_event,synh->interval);
  }

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
      double d1=window-(NOW-cycle_start_time);
 
      if(d1>0){
      double backoff=Random::uniform()*d1;
      backoff_handler.window_=window;
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
      Packet::free(pkt);
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
RMac::ProcessLargeACKNDPacket(Packet* pkt)
{
    hdr_ack_nd* ackndh=HDR_ACK_ND(pkt);
    hdr_cmn* cmh=HDR_CMN(pkt);
    time_record*  arrival_t=new time_record[TABLE_SIZE];

    memcpy((unsigned char*)arrival_t,pkt->accessdata(),sizeof(arrival_table));
     

    int  sender=ackndh->sender_addr;
    double t4=NOW;
    double t3=cmh->ts_;
   
    int myaddr=node_->address();
 
    double t2=-1.0;
    double t1=-1.0;
    

    /*
   for (int i=0;i<TABLE_SIZE;i++)
    printf("node %d  and arrival time %f\n",arrival_t[i].node_addr,arrival_t[i].arrival_time);
   
    */

    for (int i=0;i<TABLE_SIZE;i++)
    if (arrival_t[i].node_addr==myaddr)
      {
         t2=arrival_t[i].arrival_time;
         t1=arrival_t[i].sending_time;
      }

    // printf("!!!node is %d  t1= %f t2=%f t3=%f t4=%f\n",myaddr,t1,t2,t3,t4);
    delete arrival_t;

     Packet::free(pkt);
       
    if(-1.0==t2) {
      printf("rmac:ProcessLargeACKNDPacket, node %d doesn't get my(%d) ND\n",sender,myaddr);
      return;
    }
  

double latency=((t4-t1)-(t3-t2))/2.0;
bool newone=true;   
   
 for (int i=0;i<TABLE_SIZE;i++)
 if (large_latency_table[i].node_addr==sender)
      {
       large_latency_table[i].sumLatency+=latency;
       large_latency_table[i].num++;
       large_latency_table[i].last_update_time=NOW;
       large_latency_table[i].latency = large_latency_table[i].sumLatency/large_latency_table[i].num;
       newone=false;
      }
 
 if(newone)
{

    if(large_latency_table_index>=TABLE_SIZE){ 
      printf("rmac:ProcessNDPacket:arrival_table is full\n");
      return;
    }

    large_latency_table[large_latency_table_index].node_addr=sender;
    large_latency_table[large_latency_table_index].sumLatency+=latency;
    large_latency_table[large_latency_table_index].num++;
    large_latency_table[large_latency_table_index].last_update_time=NOW;
    large_latency_table[large_latency_table_index].latency = large_latency_table[large_latency_table_index].sumLatency/large_latency_table[large_latency_table_index].num;
    large_latency_table_index++;
}

 
  for (int i=0;i<large_latency_table_index;i++)
    printf("node %d and node %d  large latency is %f and number is %d\n",myaddr,large_latency_table[i].node_addr,large_latency_table[i].latency,large_latency_table[i].num);
 
 return;

}

void 
RMac::ProcessShortACKNDPacket(Packet* pkt)
{
    hdr_ack_nd* ackndh=HDR_ACK_ND(pkt);
    hdr_cmn* cmh=HDR_CMN(pkt);

    int  sender=ackndh->sender_addr;
    double t4=NOW;
    double t3=cmh->ts_;
    int myaddr=node_->address();
 
    double t2=ackndh->arrival_time;
    double t1=ackndh->ts;

double latency=((t4-t1)-(t3-t2))/2.0;
bool newone=true;   

Packet::free(pkt);
   
 for (int i=0;i<TABLE_SIZE;i++)
 if (short_latency_table[i].node_addr==sender)
      {
       short_latency_table[i].sumLatency+=latency;
       short_latency_table[i].num++;
       short_latency_table[i].last_update_time=NOW;
       short_latency_table[i].latency = 
                  short_latency_table[i].sumLatency/short_latency_table[i].num;
       newone=false;
      }
 
 if(newone)
{

    if(short_latency_table_index>=TABLE_SIZE){ 
      printf("rmac:ProcessNDPacket:arrival_table is full\n");
      return;
    }

    short_latency_table[short_latency_table_index].node_addr=sender;
    short_latency_table[short_latency_table_index].sumLatency+=latency;
    short_latency_table[short_latency_table_index].num++;
    short_latency_table[short_latency_table_index].last_update_time=NOW;
    short_latency_table[short_latency_table_index].latency = 
          short_latency_table[short_latency_table_index].sumLatency/short_latency_table[short_latency_table_index].num;
    short_latency_table_index++;
}
 for(int i=0;i<short_latency_table_index;i++)
   printf("node (%d) to node (%d) short latency is %f and number is %d\n", myaddr, short_latency_table[i].node_addr, short_latency_table[i].latency,short_latency_table[i].num); 
 
 return;

}


void 
RMac::ProcessSYN(Packet* pkt)
{
    hdr_syn* synh=HDR_SYN(pkt);
    hdr_cmn* cmh=HDR_CMN(pkt);

    int  sender=synh->sender_addr;
    double interval=synh->interval;
    double tduration=synh->duration;
      Packet::free(pkt);


    double t1=-1.0;
 for (int i=0;i<TABLE_SIZE;i++)
 if (short_latency_table[i].node_addr==sender)
     t1=short_latency_table[i].latency;

 if(t1==-1.0) {
   printf("Rmac:ProcessSYN: I receive a SYN from unknown neighbor\n");
   return; 
 }

 interval-=t1;
 double t2=next_period-(NOW-cycle_start_time);
 double d=interval-t2;
 bool newone=true;     
   
 for (int i=0;i<TABLE_SIZE;i++)
 if (period_table[i].node_addr==sender)
      {
       period_table[i].difference=d;
       period_table[i].last_update_time=NOW;
       period_table[i].duration =tduration; 
       newone=false;
      }
 
 if(newone)
{

    if(period_table_index>=TABLE_SIZE){ 
      printf("rmac:ProcessSYN:period_table is full\n");
      return;
    }


    period_table[period_table_index].node_addr=sender;
    period_table[period_table_index].difference=d;
    period_table[period_table_index].last_update_time=NOW;
    period_table[period_table_index].duration=tduration;
    period_table_index++;
}

 for(int i=0;i<period_table_index;i++)
   printf("node (%d) to node (%d) period difference  is %f \n",index_,period_table[i].node_addr, period_table[i].difference); 
 
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
 
   printf("rmac:node %d  gets a packet at time %f\n",node_->address(),NOW);
  if(dst==MAC_BROADCAST){
    
    if (cmh->ptype_==PT_ND) ProcessNDPacket(pkt); //this is ND packet

        // this is ACK_ND packet  
    if (cmh->ptype_==PT_ACK_ND) ProcessLargeACKNDPacket(pkt); 
    if (cmh->ptype_==PT_SYN) ProcessSYN(pkt);
   
    // uptarget_->recv(pkt, this);
    return;
  }

   if(dst==index_){
   if (cmh->ptype_==PT_ACK_ND) ProcessShortACKNDPacket(pkt); 
     // printf("underwaterbroadcastmac:this is my packet \n");
     //  uptarget_->recv(pkt, this);
    return;
}
    printf("rmac: this is neither broadcast nor my packet(%d), just drop it at %f\n",index_, NOW);
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

   cmh->size()=large_packet_size_;// not used here
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


