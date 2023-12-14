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

#ifndef __CPU_LVPU_LVP_UNIT_HH__
#define __CPU_LVPU_LVP_UNIT_HH__

#include <deque>
#include "base/sat_counter.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "cpu/inst_seq.hh"
#include "cpu/lvpu/lvpt.hh"
#include "cpu/static_inst.hh"
#include "params/LoadValuePredictor.hh"
#include "sim/probe/pmu.hh"
#include "sim/sim_object.hh"

namespace gem5
{
namespace load_value_prediction
{

/**
 * Basically a wrapper class to hold both the load value predictor
 * and the LVPT.
 */
class LVPredUnit : public SimObject
{
  private:
    /** The LVPT. */
    LoadValuePredictionTable LVPT;

    struct LVPredUnitStats : public statistics::Group
    {
        LVPredUnitStats(statistics::Group *parent);
        /** Stat for number of LVPU lookups (same as number of loads). */
        statistics::Scalar lvpuLookups;
        /** Stat for number of loads that were predicted. */
        statistics::Scalar predicted;
        /** Stat for number of loads predicted incorrectly. */
        statistics::Scalar predictedIncorrect;
        /** Stat for number of loads with requests exceeding 8 B in size. */
        statistics::Scalar bigLoads;
        /** Stat for number of stores with requests exceeding 8 B in size. */
        statistics::Scalar bigStores;
    } stats;

    /** Calculates the local index based on the PC. */
    inline unsigned int getLCTIndex(const Addr load_addr);

    /** Size of the load classification table. */
    const unsigned lctSize;

    /** Number of bits of the load classification table's counters. */
    const unsigned lctBits;

    /** Number of sets. */
    const unsigned lctSets;

    /** Array of counters that make up the load classification table. */
    std::vector<SatCounter8> loadClassTable;

    /** Mask to get index bits. */
    const unsigned lctIndexMask;

  public:
    typedef enum eLoadClass
    {
        UnpredictableStrong,
        UnpredictableWeak,
        Predictable,
        Constant
    } eLoadClass;

    typedef struct Result
    {
        uint64_t loadData;
        unsigned loadSize;
        eLoadClass loadClass;
    } Result;

  protected:
    /**
     * Looks up a given PC in the LVPT to get the predicted value. The PC may
     * be changed or deleted in the future, so it needs to be used immediately,
     * and/or copied for use later.
     * @param inst_PC The PC to look up.
     * @return The value at the address of the load.
     */
    void
    lvptLookup(const Addr inst_pc, const ThreadID tid, Result &result)
    {
        LVPT.lookup(inst_pc, tid, result.loadData, result.loadSize);
    }

    /**
     * Updates the LVPT with the value of a load address.
     * @param inst_PC The load's PC that will be updated.
     * @param value The load's value that will be added to the LVPT.
     */
    void
    lvptUpdate(Addr instPC, Result result, ThreadID tid)
    {
        LVPT.update(instPC, result.loadData, result.loadSize, tid);
    }

    /**
     * Updates the LCT with taken/not taken information.
     * @param inst_PC The load's PC that will be updated.
     * @param correct Set to true if the value in the LVPT matched what was at
     * the data address.
     * @param corrData The resolved data at the load data address
     * @todo Make this update flexible enough to handle a global predictor.
     */
    void lctUpdate(const Addr instPC, const bool correct);

    /** Number of bits to shift instructions by for predictor addresses. */
    const unsigned instShiftAmt;

    /**
     * Helper method to instantiate probe points belonging to this
     * object.
     *
     * @param name Name of the probe point.
     * @return A unique_ptr to the new probe point.
     */
    probing::PMUUPtr pmuProbePoint(const char *name);

    /**
     * Branches seen by the load value predictor
     *
     * @note This counter includes speculative loads.
     */
    probing::PMUUPtr ppLoads;

    /** Miss-predicted loads */
    probing::PMUUPtr ppMisses;
    /** @} */

  public:
    /**
     *  Returns the predictability of the load given the value of the
     *  LCT entry.
     *  @param pc The address of the counter.
     *  @return The prediction based on the counter value.
     */
    inline eLoadClass getLoadClass(ThreadID tid, Addr pc);

    /** Converts the eLoadClass enum to a string and return it. */
    std::string getLoadClassString(eLoadClass loadClass);

    /**
     * Predicts the value of the load.
     * @param inst The load instruction.
     * @param pc The load PC.
     * @param result The load class and lvpt entry are passed back through this
     * parameter.
     * @param tid The thread id.
     * @return Returns whether the lvpt entry is predictable.
     */
    Result
    getPrediction(const StaticInstPtr &inst, const InstSeqNum &loadSeqNum,
                  PCStateBase &pc, ThreadID tid);

    typedef LoadValuePredictorParams Params;
    /**
     * @param params The params object, that has the size of the LVPU and LVPT.
     */
    LVPredUnit(const Params &p);

    void regProbePoints() override;

    /** Perform sanity checks after a drain. */
    void drainSanityCheck() const;

    /**
     * Tells the lvpu to commit any updates until the given
     * sequence number.
     * @param done_sn The sequence number to commit any older updates up until.
     * @param tid The thread id.
     */
    void update(const InstSeqNum &done_sn, bool correct, uint64_t corr_data,
                unsigned corr_size, ThreadID tid);

    /**
     * Squashes all outstanding updates until a given sequence number.
     * @param squashed_sn The sequence number to squash any younger updates up
     * until.
     * @param tid The thread id.
     */
    void squash(const InstSeqNum &squashed_sn, ThreadID tid);

     /**
     * Squashes all outstanding updates until a given sequence number, and
     * corrects that sn's update with the proper value.
     * @param squashed_sn The sequence number to squash any younger updates up
     * until.
     * @param corr_result The correct load result.
     * @param tid The thread id.
     */
    void squash(const InstSeqNum &squashed_sn, uint64_t corr_data,
                unsigned corr_size, ThreadID tid);

    void dump();

  private:
    struct PredictorHistory
    {
        /**
         * Makes a predictor history struct that contains any
         * information needed to update the predictor and LVPT.
         */
        PredictorHistory(const InstSeqNum &load_seq_num, Addr instPC,
                          ThreadID _tid, const StaticInstPtr & inst,
                          Result _result)
            : loadSeqNum(load_seq_num), pc(instPC), tid(_tid), inst(inst),
              result(_result)
        {}

        PredictorHistory(const PredictorHistory &other) :
            loadSeqNum(other.loadSeqNum), pc(other.pc), tid(other.tid),
            inst(other.inst), result(other.result)
        {}

        bool
        operator==(const PredictorHistory &entry) const
        {
            return this->loadSeqNum == entry.loadSeqNum;
        }

        /** The sequence number for the predictor history entry. */
        InstSeqNum loadSeqNum;

        /** The PC associated with the sequence number. */
        Addr pc;

        /** Result of the load prediction. First it is predicted, and fixed
         * later if necessary
         */
        Result result;

        /** The load instruction */
        const StaticInstPtr inst;

        /** The thread id. */
        ThreadID tid;
    };

    typedef std::deque<PredictorHistory> History;

    /** Number of the threads for which the load history is maintained. */
    const unsigned numThreads;

    /**
     * The per-thread predictor history. This is used to update the predictor
     * as instructions are committed, or restore it to the proper state after
     * a squash.
     */
    std::vector<History> loadPredHist;
};

} // namespace load_value_prediction
} // namespace gem5

#endif // __CPU_LVPU_LVP_UNIT_HH__
