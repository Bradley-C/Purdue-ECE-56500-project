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

// #include "params/LoadValuePredictor.hh"
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
  public:
      typedef LVPUParams Params;
    /**
     * @param params The params object, that has the size of the BP and BTB.
     */
    LVPredUnit(const Params &p);

    void regProbePoints() override;

    /** Perform sanity checks after a drain. */
    void drainSanityCheck() const;

    /**
     * Predicts the value of the load.
     * @param inst The load instruction.
     * @param value The predicted value is passed back through this parameter.
     * @param tid The thread id.
     * @return Returns whether the lvpt entry was valid or not.
     */
    bool predict(const StaticInstPtr &inst, const InstSeqNum &seqNum,
                 uint8_t *value, ThreadID tid);

    /**
     * Tells the load value predictor to commit any updates until the given
     * sequence number.
     * @param done_sn The sequence number to commit any older updates up until.
     * @param tid The thread id.
     */
    void update(const InstSeqNum &done_sn, ThreadID tid);

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
     * @param corr_value The correct branch target.
     * @param tid The thread id.
     */
    void squash(const InstSeqNum &squashed_sn,
                const PCStateBase &corr_target,
                ThreadID tid);

    /**
     * @param lct Pointer to the load classification table object. The
     * predictor will need to update any state and delete the object.
     */
    virtual void squash(ThreadID tid, void *lct) = 0;

    /**
     * Looks up a given PC in the LVP to see if it ought to be predicted.
     * @param inst_PC The PC to look up.
     * @param lct Pointer that will be set to an object that
     * has the load predictor state associated with the lookup.
     * @return Whether the load is predictable.
     */
    virtual bool lookup(ThreadID tid, Addr instPC, void * &lct) = 0;

     /**
     * If a load cannot be predicted, because the LVPT value is invalid,
     * this function sets the appropriate counter in the load classification
     * table to not predictable.
     * @param inst_PC The PC to look up the load in the LCT.
     * @param lct Pointer that will be set to an object that
     * has the load predictor state associated with the lookup.
     */
    virtual void lvptUpdate(ThreadID tid, Addr instPC, void * &lct) = 0;

    /**
     * Looks up a given PC in the LVPT to see if a valid entry exists.
     * @param inst_PC The PC to look up.
     * @return Whether the LVPT contains the given PC.
     */
    bool LVPTValid(Addr instPC) { return BTB.valid(instPC, 0); }

    /**
     * Looks up a given PC in the LVPT to get the predicted value. The PC may
     * be changed or deleted in the future, so it needs to be used immediately,
     * and/or copied for use later.
     * @param inst_PC The PC to look up.
     * @return The value at the address of the load.
     */
    const uint64_t value*
    LVPTLookup(Addr inst_pc)
    {
        return LVPT.lookup(inst_pc, 0);
    }

    /**
     * Updates the LVPU with taken/not taken information.
     * @param inst_PC The load's PC that will be updated.
     * @param predicted Whether the load was predicted on.
     * @param lct Pointer to the load predictor state that is
     * associated with the load lookup that is being updated.
     * @param squashed Set to true when this function is called during a
     * squash operation.
     * @param inst Static instruction information
     * @param corrValue The resolved value at the load data address (only
     * needed for squashed loads)
     * @todo Make this update flexible enough to handle a global predictor.
     */
    virtual void update(ThreadID tid, Addr instPC, bool predicted,
                   void *lct, bool squashed,
                   const StaticInstPtr &inst, uint8_t *corrValue) = 0;
    /**
     * Updates the LVPT with the value of a load address.
     * @param inst_PC The load's PC that will be updated.
     * @param value The load's value that will be added to the LVPT.
     */
    void
    LVPTUpdate(Addr instPC, const uint8_t *value)
    {
        LVPT.update(instPC, value, 0);
    }


    void dump();

  private:
    struct PredictorHistory
    {
        /**
         * Makes a predictor history struct that contains any
         * information needed to update the predictor, LVPT, and CVU.
         */
        PredictorHistory(const InstSeqNum &seq_num, Addr instPC,
                         bool _predicted, ThreadID _tid,
                         const StaticInstPtr & inst)
            : seqNum(seq_num), pc(instPC), tid(_tid),
              predicted(_predicted), inst(inst)
        {}

        PredictorHistory(const PredictorHistory &other) :
            seqNum(other.seqNum), pc(other.pc), RASIndex(other.RASIndex),
            tid(other.tid), predicted(other.predicted),
            value(other.value), inst(other.inst)
        {
            set(RASTarget, other.RASTarget);
        }

        bool
        operator==(const PredictorHistory &entry) const
        {
            return this->seqNum == entry.seqNum;
        }

        /** The sequence number for the predictor history entry. */
        InstSeqNum seqNum;

        /** The PC associated with the sequence number. */
        Addr pc;

        /** The thread id. */
        ThreadID tid;

        /** Whether or not it was predicted. */
        bool predicted;

        /** Value of the load. First it is predicted, and fixed later
         *  if necessary
         */
        uint8_t *value = nullptr;

        /** The load instrction */
        const StaticInstPtr inst;
    };

    typedef std::deque<PredictorHistory> History;

    /** Number of the threads for which the branch history is maintained. */
    const unsigned numThreads;


    /**
     * The per-thread predictor history. This is used to update the predictor
     * as instructions are committed, or restore it to the proper state after
     * a squash.
     */
    std::vector<History> predHist;

    /** The BTB. */
    LoadValuePredictionTable LVPT;

    struct LVPredUnitStats : public statistics::Group
    {
        LVPredUnitStats(statistics::Group *parent);
        /** Stat for number of LVPT lookups. */
        statistics::Scalar LVPTLookups;
        /** Stat for number of loads predicted incorrectly. */
        statistics::Scalar predictedIncorrect;
    } stats;

  protected:
    /** Number of bits to shift instructions by for predictor addresses. */
    const unsigned instShiftAmt;

    /**
     * @{
     * @name PMU Probe points.
     */

    /**
     * Helper method to instantiate probe points belonging to this
     * object.
     *
     * @param name Name of the probe point.
     * @return A unique_ptr to the new probe point.
     */
    probing::PMUUPtr pmuProbePoint(const char *name);


    /**
     * Branches seen by the branch predictor
     *
     * @note This counter includes speculative branches.
     */
    probing::PMUUPtr ppLoads;

    /** Miss-predicted loads */
    probing::PMUUPtr ppMisses;

    /** @} */
};

} // namespace load_value_prediction
} // namespace gem5

#endif // __CPU_LVPU_LVP_UNIT_HH__
