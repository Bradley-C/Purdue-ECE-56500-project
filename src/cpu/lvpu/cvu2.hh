/*
 * Copyright (c) 2011-2012, 2014 ARM Limited
 * Copyright (c) 2010 The University of Edinburgh
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __CPU_LVPU_CVU_UNIT_HH__
#define __CPU_LVPU_CVU_UNIT_HH__


#include <deque>

#include "base/sat_counter.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "cpu/inst_seq.hh"
#include "cpu/static_inst.hh"
#include "params/ConstantVerificationUnit.hh"
#include "sim/probe/pmu.hh"
#include "sim/sim_object.hh"

namespace gem5
{
namespace load_value_prediction
{

/**
 * Basically a wrapper class to hold both the constant value unit
 */
class ConstantVUnit : public SimObject
{

  private:

    struct CVUUnitStats : public statistics::Group
    {
        CVUUnitStats(statistics::Group *parent);
        /** Stat for number of LVPU lookups (same as number of loads). */
        statistics::Scalar cvuLoadLookups;
        statistics::Scalar cvuStoreLookups;

        /** Stat for number of instuctions & data Addr that were matched. */
        statistics::Scalar loadMatched;
        /** Stat for number of instuctions & data Addr that had no match. */
        statistics::Scalar loadMatchedIncorrect;
        /** Stat for number of store instuction Addr that were matched. */
       // statistics::Scalar loadMatched;
    } stats;

    /** Returns the index into the CVU, based on the load's PC.
     *  @param inst_PC The load to look up.
     *  @param tid The thread id.
     *  @return Returns the index into the CVU.
     */
    inline unsigned getIndexCVU(Addr instPC, ThreadID tid);

    /** Check is given PC entry is Valid
     *  @param inst_PC The entry to look up.
     *  @param tid The thread id.
     *  @return Returns if the entry is valid
     */
    bool checkValid (Addr instPC, ThreadID tid);

    struct CVUEntry
    {
        /** The entry's value. */
        uint64_t value;

        /** The entry's thread id. */
        ThreadID tid;

        /** Whether or not the entry is valid. */
        bool valid = false;
    };

    /** The actual CVU. */
    std::vector<std::vector<CVUEntry>> cvu; // Declare a 2D array
    //CVUEntry cvu[numEntries][numAddrperEntry]; // Declare a 2D array

    /** The number of entries in the CVU. */
    const unsigned numEntries;

    /** The number of memory addresses stored per entry in the CVU. */
    const unsigned numAddrperEntry;
    /** Number of bits to shift PC when calculating index. */
    const unsigned instShiftAmt;

  protected:


    /** Do CAM search for given PC and Load Addr, used to pass cache for loads
     *  @param inst_pc The PC address of the store being updated.
     *  @param tid The thread id.
     *  @return CVUReturn that tells the LVPT/LCT how to update
     */
   // CVUReturn getReturnStruct(Addr inst_pc, ThreadID tid);

    /**
     * Helper method to instantiate probe points belonging to this
     * object.
     *
     * @param name Name of the probe point.
     * @return A unique_ptr to the new probe point.
     */
    probing::PMUUPtr pmuProbePoint(const char *name);

    /**
     * Branches seen by the constant value unit
     *
     * @note This counter includes speculative loads.
     */
    probing::PMUUPtr ppLoads;

    /** Miss-predicted loads */
    probing::PMUUPtr ppMisses;
    /** @} */

    public:

    typedef ConstantVerificationUnitParams Params;
    /**
     * @param params The params object, that has the size of the LVPU and LVPT.
     */
    ConstantVUnit(const Params &p);

    struct CVUReturn
    {
        /** The entry's index to clear */
        Addr pc = 0;

        /** The entry's value to replace old*/
        uint64_t value = 0;

        /** Whether or not the replacement is valid. */
        bool clear = false;

        bool update = false;
    };

    /** Updates the CVU an entry with a new addr when new constant
     * found
     *  @param inst_pc The PC address of the load being updated.
     *  @param new_value The address that was loaded.
     *  @param tid The thread id.
     */
    void updateEntry( PCStateBase &pc,
    uint64_t new_value, ThreadID tid);

    /** Clears the CVU of valid entry when data addr matches a given
     *  entry for stores
     *  @param inst_pc The PC address of the store being updated.
     *  @param data_addr The address that is stored to.
     *  @param new_addr The value that was stored.
     *  @param tid The thread id.
     *  @return CVUReturn that tells the LVPT/LCT how to update
     */
    ConstantVUnit::CVUReturn storeClear(PCStateBase &inst_pc,
    uint64_t data_addr, uint64_t new_addr, ThreadID tid);

    /** Do CAM search for given PC and Load Addr, used to pass cache
     *  for loads
     *  @param inst_pc The PC address of the store being updated.
     *  @param data_addr The address that is being loaded.
     *  @param tid The thread id.
     *  @return CVUReturn that tells the LVPT/LCT how to update
     */
    ConstantVUnit::CVUReturn addrMatch(PCStateBase &inst_pc,
    uint64_t data_addr, ThreadID tid);

    /**
     * Tells the CVU to commit any updates until the given
     * sequence number.
     * @param done_sn The sequence number to commit any older updates up until.
     * @param tid The thread id.
     */
    void update(const InstSeqNum &done_sn, uint64_t new_value, ThreadID tid);

    /**
     * Squashes all outstanding updates until a given sequence number.
     * @param squashed_sn The sequence number to squash any younger updates up
     * until.
     * @param tid The thread id.
     */
    void squash(const InstSeqNum &squashed_sn, ThreadID tid);

    void dump();

    void regProbePoints() override;

    /** Perform sanity checks after a drain. */
    void drainSanityCheck() const;

  private:
    struct PredictorHistory
    {
        /**
         * Makes a CVU history struct that contains any
         * information needed to update the CVU.
         */
        PredictorHistory(const InstSeqNum &seq_num, Addr instPC,
                        std::vector<std::vector<CVUEntry>> _CVUTable,
                         ThreadID _tid, const StaticInstPtr & inst,
                         uint64_t _data)
            : seqNum(seq_num), pc(instPC), CVUTable(_CVUTable),
              tid(_tid), inst(inst), data(_data)
        {}

        PredictorHistory(const PredictorHistory &other) :
            seqNum(other.seqNum), pc(other.pc),
            CVUTable(other.CVUTable), tid(other.tid), inst(other.inst),
            data(other.data)
        {}

        bool
        operator==(const PredictorHistory &entry) const
        {
            return this->seqNum == entry.seqNum;
        }

        /** The sequence number for the CVU history entry. */
        InstSeqNum seqNum;

        /** The PC associated with the sequence number. */
        Addr pc;

        /** Value of the load. First it is predicted, and fixed later
         *  if necessary
         */
        uint64_t data;

        /** The load instruction */
        const StaticInstPtr inst;

        /** The load classification*/
        std::vector<std::vector<CVUEntry>> CVUTable ;

        /** The thread id. */
        ThreadID tid;
    };

    typedef std::deque<PredictorHistory> History;

    /** Number of the threads for which the load history is maintained. */
    const unsigned numThreads;

    /**
     * The per-thread CVU history. This is used to update the CVU
     * as instructions are committed, or restore it to the proper state after
     * a squash.
     */
    std::vector<History> predHist;
};

} // namespace load_value_prediction
} // namespace gem5

#endif // __CPU_LVPU_CVU_UNIT_HH__
