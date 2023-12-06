/*
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

#ifndef __CPU_LVPU_LVPT_HH__
#define __CPU_LVPU_LVPT_HH__

#include "arch/generic/pcstate.hh"
#include "base/logging.hh"
#include "base/types.hh"
#include "config/the_isa.hh"

namespace gem5
{

namespace load_value_prediction
{

class LoadValuePredictionTable
{
  private:
    struct LVPTEntry
    {
        /** The entry's data. */
        uint64_t data;

        /** The entry's thread id. */
        ThreadID tid;
    };

    /** Returns the index into the LVPT, based on the load's PC.
     *  @param inst_PC The load to look up.
     *  @return Returns the index into the LVPT.
     */
    inline unsigned getIndex(Addr instPC, ThreadID tid);

    /** The actual LVPT. */
    std::vector<LVPTEntry> LVPT;

    /** The number of entries in the LVPT. */
    unsigned numEntries;

    /** The index mask. */
    unsigned idxMask;

    /** Number of bits to shift PC when calculating index. */
    unsigned instShiftAmt;

    /** Log2 NumThreads used for hashing threadid */
    unsigned log2NumThreads;

  public:
    /** Creates an LVPT with the given number of entries and instruction offset
     *  amount.
     *  @param numEntries Number of entries for the LVPT.
     *  @param instShiftAmt Offset amount for instructions to ignore alignment.
     */
    LoadValuePredictionTable(unsigned numEntries,
                             unsigned instShiftAmt, unsigned numThreads);

    /** Get a value from an LVPT entry. */
    void lookup(Addr inst_pc, ThreadID tid, uint64_t &data);

    /** Updates the LVPT with the mispredicted value of a load.
     *  @param inst_pc The address of the load being updated.
     *  @param new_data The data at the address that was loaded.
     *  @param tid The thread id.
     */
    void update(Addr inst_pc, const uint64_t new_data, ThreadID tid);
};

} // namespace load_value_prediction
} // namespace gem5

#endif // __CPU_LVPU_LVPT_HH__
