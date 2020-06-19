#ifndef __MEM_MMU_HH__
#define __MEM_MMU_HH__

#include <iostream>
#include <sstream>
#include <string>

#include "mem/spm/spm_class/spm.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/protocol/Types.hh"
#include "params/MMU.hh"

class ATT;

class MMU : public SimObject
{
  public:

  	/**
	 * cpu-side端口，用于接收请求
	 */
	class MMUSlavePort : public QueueSlavePort
	{
	
	  private:
	    
	    RespPacketQueue _respQueue;
	    MMU *mmu;
		
	  public:
	  
	    MMUSlavePort(const std::string& _name, MMU* _mmu, PortID id);
	    bool recvTimingReq(PacketPtr);

		
	  protected:
		
		bool recvTimingSnoopResp(PacketPtr pkt) override;
		
		Tick recvAtomic(PacketPtr pkt)
		{
			panic("MMU::MMUSideSlavePort::recvAtomic() not implemented!\n");
		}
		
		bool tryTiming(PacketPtr pkt) override;
		
		void recvFunctional(PacketPtr pkt) override;
		
		void recvRespRetry() override;
		
		AddrRangeList getAddrRanges() const override;

	  private:

	  	bool isPhysMemAddress(Addr addr) const;
	};

	/**
	 * mem-side端口，将请求重定向到相应的cache或spm
	 */
	class MMUMasterPort : public QueueMasterPort
	{
	
	  private:
	  
	  	ReqPacketQueue _reqQueue;
	  	SnoopRespPacketQueue _snoopRespQueue;
	    MMU *mmu;
		
	  public:
	  
	    /**
		 * Constructor. Just calls the superclass constructor
		 */
		MMUMasterPort(const std::string& _name, MMU* _mmu, PortID id);
		
		bool recvTimingResp(PacketPtr pkt);
		
	  protected:
	  
	  
		Tick recvAtomicSnoop(PacketPtr pkt) override;
		
		void recvTimingSnoopReq(PacketPtr pkt) override;
		
		void recvFunctionalSnoop(PacketPtr pkt) override;
		
		void recvReqRetry() override;
		
		void recvRetrySnoopResp() override;
		
		void recvRangeChange() override;
		
		bool isSnooping() const override;

	  private:

	  	bool isPhysMemAddress(Addr addr) const;
	};

	typedef MMUParams Params;
	MMU(const Params *p);

	SPM *spm_ptr;

	/// Instantiation of the CPU-side port
	MMUSlavePort cpuPort;
	
	/// Instantiation of the cache-side port
	MMUMasterPort cachePort;
	MMUMasterPort spmPort;

	virtual BaseMasterPort& getMasterPort(const std::string& if_name,
										  PortID idx = InvalidPortID);
	virtual BaseSlavePort& getSlavePort(const std::string& if_name,
										PortID idx = InvalidPortID); 

	void setSPM(SPM *_spm);

	// aligns the address to virtual page boundaried
	static Addr spmPageAlign(Addr v_addr)		{ return spmPageAlignDown(v_addr); }
	static Addr spmPageAlignDown(Addr v_addr)	{ return (v_addr & ~(Addr(m_page_size_bytes - 1))); }
	static Addr spmPageAlignUp(Addr v_addr)		{ return (v_addr + (m_page_size_bytes - 1)) &
														 ~(Addr(m_page_size_bytes - 1)); }

	static int getPageSizeBytes() { return m_page_size_bytes; }
	int getSPMSizePage() { return spm_ptr->getSize() / m_page_size_bytes; }

	NodeID getNodeID() const { return m_machineID.num; }

  private:

  	static int m_page_size_bytes;

  public:

  	ATT *my_att;

  private:

  	bool handleTimingReq(PacketPtr pkt);
}; 

#endif // __MEM_MMU_HH__