#ifndef ns_dataprocessor_h
#define ns_dataprocessor_h

#include "agent.h"
#include "tclcl.h"
#include "packet.h"
#include "address.h"
#include "ip.h"

struct hdr_data{
	double send_time;
 	double rcv_time;	// when ping arrived to receiver
 	int seq;		// sequence number

	// Header access methods
	static int offset_; // required by PacketHeaderManager
	inline static int& offset() { return offset_; }
	inline static hdr_datagenerator* access(const Packet* p) {
		return (hdr_datagenerator*) p->access(offset_);
	}
};

class DataGeneratorAgent : public Agent {
public:
	DataGeneratorAgent();
 	int seq;	// a send sequence number
        double cum_delay;
        double interval_;
        void Terminate();
        void stop();
	void start();
	virtual int command(int argc, const char*const* argv);
	virtual void recv(Packet*, Handler*);
        virtual void sendpkt();
};

#endif // ns_uwping_h
