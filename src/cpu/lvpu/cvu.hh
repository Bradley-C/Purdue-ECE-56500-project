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

#ifndef __CPU_LVPU_CVU_HH__
#define __CPU_LVPU_CVU_HH__

#include "arch/generic/pcstate.hh"
#include "base/logging.hh"
#include "base/types.hh"
#include "config/the_isa.hh"

namespace gem5
{

namespace load_value_prediction
{

class ConstantVerificationUnit
{
  private:
    struct CVUEntry
    {
        /** The entry's value. */
        uint64_t value;

        /** The entry's thread id. */
        ThreadID tid;

        /** Whether or not the entry is valid. */
        bool valid = false;
    };

    struct CVUReturn
    {
        /** The entry's index to clear */
        uint32_t index;

        /** The entry's value to replace old*/
        uint32_t value;

        /** Whether or not the replacement is valid. */
        bool clear = false;
    };

  public:
    /** Creates an CVU with the given number of entries, number of bits per
     *  tag, and instruction offset amount.
     *  @param numEntries Number of entries for the LVPT.
     *  @param tagBits Number of bits for each tag in the LVPT.
     *  @param instShiftAmt Offset amount for instructions to ignore alignment.
     */
    ConstantVerificationUnit(unsigned numEntries, unsigned tagBits,
                unsigned instShiftAmt, unsigned numThreads);

    void reset();

    /** Checks if a value is in the CVU.
     *  @param inst_PC The PC address of the load to look up.
     *  @param tid The thread id.
     *  @return Whether or not the value exists in the CVU (i.e. the entry is
     *  not still initialized to zero).W
     */
    bool valid(Addr instPC, ThreadID tid);

    /** Updates the LVPT with the mispredicted value of a load.
     *  @param inst_pc The PC address of the load being updated.
     *  @param new_value The address that was loaded.
     *  @param tid The thread id.
     */
    void update(Addr inst_pc, const uint32_t new_value, ThreadID tid);

    /** Clears the CVU of valid entry when data addr matches a given entry.
     *  @param inst_pc The PC address of the store being updated.
     *  @param data_addr The address that is stored to.
     *  @param new_addr The value that was stored.
     *  @param tid The thread id.
     *  @return CVUReturn to be sent back to the LVPT/LCT in the Fetch2 stage
     */
    void store_clear(Addr inst_pc, const uint32_t data_addr,
                      const uint32_t new_addr, ThreadID tid);

  private:
    /** Returns the index into the CVU, based on the load's PC.
     *  @param inst_PC The load to look up.
     *  @return Returns the index into the CVU.
     */
    inline unsigned getIndex(Addr instPC, ThreadID tid);


    /** The actual CVU. */
    std::vector<CVUEntry> cvu;

    /** The number of entries in the CVU. */
    unsigned numEntries;

    /** The index mask. */
    unsigned idxMask;

    /** Number of bits to shift PC when calculating index. */
    unsigned instShiftAmt;

    /** Number of bits to shift PC when calculating tag. */
    unsigned tagShiftAmt;

    /** Log2 NumThreads used for hashing threadid */
    unsigned log2NumThreads;
};

} // namespace load_value_prediction
} // namespace gem5

#endif // __CPU_LVPU_CVU_HH__
