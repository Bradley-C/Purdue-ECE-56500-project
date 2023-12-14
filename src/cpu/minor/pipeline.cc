/*
 * Copyright (c) 2013-2014, 2020 ARM Limited
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

#include "cpu/minor/pipeline.hh"

#include <algorithm>
#include "cpu/minor/decode.hh"
#include "cpu/minor/execute.hh"
#include "cpu/minor/fetch1.hh"
#include "cpu/minor/fetch2.hh"
#include "debug/Drain.hh"
#include "debug/MinorCPU.hh"
#include "debug/MinorTrace.hh"
#include "debug/Quiesce.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(Minor, minor);
namespace minor
{

#if ENABLE_16STAGES
Pipeline::Pipeline(MinorCPU &cpu_, const BaseMinorCPUParams &params) :
    Ticked(cpu_, &(cpu_.BaseCPU::baseStats.numCycles)),
    cpu(cpu_),
    allow_idling(params.enableIdling),
    f1ToF2(cpu.name() + ".f1ToF2", "lines",
        params.fetch1ToFetch2ForwardDelay),
    f2ToF1(cpu.name() + ".f2ToF1", "prediction",
        params.fetch1ToFetch2BackwardDelay, true),
    f2ToD(cpu.name() + ".f2ToD", "insts",
        params.fetch2ToDecodeForwardDelay),
    dToD2(cpu.name() + ".dToD2", "insts",
        params.fetch2ToDecodeForwardDelay),
    d2ToD3(cpu.name() + ".d2ToD3", "insts",
        params.fetch2ToDecodeForwardDelay),
    d3ToD4(cpu.name() + ".d3ToD4", "insts",
        params.fetch2ToDecodeForwardDelay),
    d4ToD5(cpu.name() + ".d4ToD5", "insts",
        params.fetch2ToDecodeForwardDelay),
    d5ToD6(cpu.name() + ".d5ToD6", "insts",
        params.fetch2ToDecodeForwardDelay),
    d6ToD7(cpu.name() + ".d6ToD7", "insts",
        params.fetch2ToDecodeForwardDelay),
    d7ToD8(cpu.name() + ".d7ToD8", "insts",
        params.fetch2ToDecodeForwardDelay),
    d8ToD9(cpu.name() + ".d8ToD9", "insts",
        params.fetch2ToDecodeForwardDelay),
    d9ToD10(cpu.name() + ".d9ToD10", "insts",
        params.fetch2ToDecodeForwardDelay),
    d10ToD11(cpu.name() + ".d10ToD11", "insts",
        params.fetch2ToDecodeForwardDelay),
    d11ToD12(cpu.name() + ".d11ToD12", "insts",
        params.fetch2ToDecodeForwardDelay),
    d12ToD13(cpu.name() + ".d12ToD13", "insts",
        params.fetch2ToDecodeForwardDelay),
    d13ToD14(cpu.name() + ".d13ToD14", "insts",
        params.fetch2ToDecodeForwardDelay),
    d14ToD15(cpu.name() + ".d14ToD15", "insts",
        params.fetch2ToDecodeForwardDelay),
    d15ToD16(cpu.name() + ".d15ToD16", "insts",
        params.fetch2ToDecodeForwardDelay),
    d16ToE(cpu.name() + ".d16ToE", "insts",
        params.decodeToExecuteForwardDelay),
    eToF1(cpu.name() + ".eToF1", "branch",
        params.executeBranchDelay),
    execute(cpu.name() + ".execute", cpu, params,
        d16ToE.output(), eToF1.input()),
    decode16(cpu.name() + ".decode16", cpu, params,
        d15ToD16.output(), d16ToE.input(), execute.inputBuffer),
    decode15(cpu.name() + ".decode15", cpu, params,
        d14ToD15.output(), d15ToD16.input(), decode16.inputBuffer),
    decode14(cpu.name() + ".decode14", cpu, params,
        d13ToD14.output(), d14ToD15.input(), decode15.inputBuffer),
    decode13(cpu.name() + ".decode13", cpu, params,
        d12ToD13.output(), d13ToD14.input(), decode14.inputBuffer),
    decode12(cpu.name() + ".decode12", cpu, params,
        d11ToD12.output(), d12ToD13.input(), decode13.inputBuffer),
    decode11(cpu.name() + ".decode11", cpu, params,
        d10ToD11.output(), d11ToD12.input(), decode12.inputBuffer),
    decode10(cpu.name() + ".decode10", cpu, params,
        d9ToD10.output(), d10ToD11.input(), decode11.inputBuffer),
    decode9(cpu.name() + ".decode9", cpu, params,
        d8ToD9.output(), d9ToD10.input(), decode10.inputBuffer),
    decode8(cpu.name() + ".decode8", cpu, params,
        d7ToD8.output(), d8ToD9.input(), decode9.inputBuffer),
    decode7(cpu.name() + ".decode7", cpu, params,
        d6ToD7.output(), d7ToD8.input(), decode8.inputBuffer),
    decode6(cpu.name() + ".decode6", cpu, params,
        d5ToD6.output(), d6ToD7.input(), decode7.inputBuffer),
    decode5(cpu.name() + ".decode5", cpu, params,
        d4ToD5.output(), d5ToD6.input(), decode6.inputBuffer),
    decode4(cpu.name() + ".decode4", cpu, params,
        d3ToD4.output(), d4ToD5.input(), decode5.inputBuffer),
    decode3(cpu.name() + ".decode3", cpu, params,
        d2ToD3.output(), d3ToD4.input(), decode4.inputBuffer),
    decode2(cpu.name() + ".decode2", cpu, params,
        dToD2.output(), d2ToD3.input(), decode3.inputBuffer),
    decode(cpu.name() + ".decode", cpu, params,
        f2ToD.output(), dToD2.input(), decode2.inputBuffer),
    fetch2(cpu.name() + ".fetch2", cpu, params,
        f1ToF2.output(), eToF1.output(), f2ToF1.input(), f2ToD.input(),
        decode.inputBuffer),
    fetch1(cpu.name() + ".fetch1", cpu, params,
        eToF1.output(), f1ToF2.input(), f2ToF1.output(), fetch2.inputBuffer),
    activityRecorder(cpu.name() + ".activity", Num_StageId,
        /* The max depth of inter-stage FIFOs */
        std::max(params.fetch1ToFetch2ForwardDelay,
        std::max(params.fetch2ToDecodeForwardDelay,
        std::max(params.decodeToExecuteForwardDelay,
        std::max(params.decodeToExecuteForwardDelay,
        std::max(params.decodeToExecuteForwardDelay,
        std::max(params.decodeToExecuteForwardDelay,
        std::max(params.decodeToExecuteForwardDelay,
        std::max(params.decodeToExecuteForwardDelay,
        std::max(params.decodeToExecuteForwardDelay,
        std::max(params.decodeToExecuteForwardDelay,
        std::max(params.decodeToExecuteForwardDelay,
        std::max(params.decodeToExecuteForwardDelay,
        std::max(params.decodeToExecuteForwardDelay,
        std::max(params.decodeToExecuteForwardDelay,
        std::max(params.decodeToExecuteForwardDelay,
        std::max(params.decodeToExecuteForwardDelay,
        std::max(params.decodeToExecuteForwardDelay,
        std::max(params.decodeToExecuteForwardDelay,
        params.executeBranchDelay))))))))))))))))))),
    needToSignalDrained(false)
{
    if (params.fetch1ToFetch2ForwardDelay < 1) {
        fatal("%s: fetch1ToFetch2ForwardDelay must be >= 1 (%d)\n",
            cpu.name(), params.fetch1ToFetch2ForwardDelay);
    }

    if (params.fetch2ToDecodeForwardDelay < 1) {
        fatal("%s: fetch2ToDecodeForwardDelay must be >= 1 (%d)\n",
            cpu.name(), params.fetch2ToDecodeForwardDelay);
    }

    if (params.decodeToExecuteForwardDelay < 1) {
        fatal("%s: decodeToExecuteForwardDelay must be >= 1 (%d)\n",
            cpu.name(), params.decodeToExecuteForwardDelay);
    }

    if (params.executeBranchDelay < 1) {
        fatal("%s: executeBranchDelay must be >= 1\n",
            cpu.name(), params.executeBranchDelay);
    }
}
#else
Pipeline::Pipeline(MinorCPU &cpu_, const BaseMinorCPUParams &params) :
    Ticked(cpu_, &(cpu_.BaseCPU::baseStats.numCycles)),
    cpu(cpu_),
    allow_idling(params.enableIdling),
    f1ToF2(cpu.name() + ".f1ToF2", "lines",
        params.fetch1ToFetch2ForwardDelay),
    f2ToF1(cpu.name() + ".f2ToF1", "prediction",
        params.fetch1ToFetch2BackwardDelay, true),
    f2ToD(cpu.name() + ".f2ToD", "insts",
        params.fetch2ToDecodeForwardDelay),
    dToE(cpu.name() + ".dToE", "insts",
        params.decodeToExecuteForwardDelay),
    eToF1(cpu.name() + ".eToF1", "branch",
        params.executeBranchDelay),
    execute(cpu.name() + ".execute", cpu, params,
        dToE.output(), eToF1.input()),
    decode(cpu.name() + ".decode", cpu, params,
        f2ToD.output(), dToE.input(), execute.inputBuffer),
    fetch2(cpu.name() + ".fetch2", cpu, params,
        f1ToF2.output(), eToF1.output(), f2ToF1.input(), f2ToD.input(),
        decode.inputBuffer),
    fetch1(cpu.name() + ".fetch1", cpu, params,
        eToF1.output(), f1ToF2.input(), f2ToF1.output(), fetch2.inputBuffer),
    activityRecorder(cpu.name() + ".activity", Num_StageId,
        /* The max depth of inter-stage FIFOs */
        std::max(params.fetch1ToFetch2ForwardDelay,
        std::max(params.fetch2ToDecodeForwardDelay,
        std::max(params.decodeToExecuteForwardDelay,
        params.executeBranchDelay)))),
    needToSignalDrained(false)
{
    if (params.fetch1ToFetch2ForwardDelay < 1) {
        fatal("%s: fetch1ToFetch2ForwardDelay must be >= 1 (%d)\n",
            cpu.name(), params.fetch1ToFetch2ForwardDelay);
    }

    if (params.fetch2ToDecodeForwardDelay < 1) {
        fatal("%s: fetch2ToDecodeForwardDelay must be >= 1 (%d)\n",
            cpu.name(), params.fetch2ToDecodeForwardDelay);
    }

    if (params.decodeToExecuteForwardDelay < 1) {
        fatal("%s: decodeToExecuteForwardDelay must be >= 1 (%d)\n",
            cpu.name(), params.decodeToExecuteForwardDelay);
    }

    if (params.executeBranchDelay < 1) {
        fatal("%s: executeBranchDelay must be >= 1\n",
            cpu.name(), params.executeBranchDelay);
    }
}
#endif

#if ENABLE_16STAGES
void
Pipeline::minorTrace() const
{
    fetch1.minorTrace();
    f1ToF2.minorTrace();
    f2ToF1.minorTrace();
    fetch2.minorTrace();
    f2ToD.minorTrace();
    decode.minorTrace();
    dToD2.minorTrace();
    decode2.minorTrace();
    d2ToD3.minorTrace();
    decode3.minorTrace();
    d3ToD4.minorTrace();
    decode4.minorTrace();
    d4ToD5.minorTrace();
    decode5.minorTrace();
    d5ToD6.minorTrace();
    decode6.minorTrace();
    d6ToD7.minorTrace();
    decode7.minorTrace();
    d7ToD8.minorTrace();
    decode8.minorTrace();
    d8ToD9.minorTrace();
    decode9.minorTrace();
    d9ToD10.minorTrace();
    decode10.minorTrace();
    d10ToD11.minorTrace();
    decode11.minorTrace();
    d11ToD12.minorTrace();
    decode12.minorTrace();
    d12ToD13.minorTrace();
    decode13.minorTrace();
    d13ToD14.minorTrace();
    decode14.minorTrace();
    d14ToD15.minorTrace();
    decode15.minorTrace();
    d15ToD16.minorTrace();
    decode16.minorTrace();
    d16ToE.minorTrace();
    execute.minorTrace();
    eToF1.minorTrace();
    activityRecorder.minorTrace();
}
#else
void
Pipeline::minorTrace() const
{
    fetch1.minorTrace();
    f1ToF2.minorTrace();
    f2ToF1.minorTrace();
    fetch2.minorTrace();
    f2ToD.minorTrace();
    decode.minorTrace();
    dToE.minorTrace();
    execute.minorTrace();
    eToF1.minorTrace();
    activityRecorder.minorTrace();
}
#endif

#if ENABLE_16STAGES
void
Pipeline::evaluate()
{
    /** We tick the CPU to update the BaseCPU cycle counters */
    cpu.tick();

    /* Note that it's important to evaluate the stages in order to allow
     *  'immediate', 0-time-offset TimeBuffer activity to be visible from
     *  later stages to earlier ones in the same cycle */
    execute.evaluate();
    decode16.evaluate16();
    decode15.evaluate15();
    decode14.evaluate14();
    decode13.evaluate13();
    decode12.evaluate12();
    decode11.evaluate11();
    decode10.evaluate10();
    decode9.evaluate9();
    decode8.evaluate8();
    decode7.evaluate7();
    decode6.evaluate6();
    decode5.evaluate5();
    decode4.evaluate4();
    decode3.evaluate3();
    decode2.evaluate2();
    decode.evaluate1();
    fetch2.evaluate();
    fetch1.evaluate();

    if (debug::MinorTrace)
        minorTrace();

    /* Update the time buffers after the stages */
    f1ToF2.evaluate();
    f2ToF1.evaluate();
    f2ToD.evaluate();
    dToD2.evaluate();
    d2ToD3.evaluate();
    d3ToD4.evaluate();
    d4ToD5.evaluate();
    d5ToD6.evaluate();
    d6ToD7.evaluate();
    d7ToD8.evaluate();
    d8ToD9.evaluate();
    d9ToD10.evaluate();
    d10ToD11.evaluate();
    d11ToD12.evaluate();
    d12ToD13.evaluate();
    d13ToD14.evaluate();
    d14ToD15.evaluate();
    d15ToD16.evaluate();
    d16ToE.evaluate();
    eToF1.evaluate();

    /* The activity recorder must be be called after all the stages and
     *  before the idler (which acts on the advice of the activity recorder */
    activityRecorder.evaluate();

    if (allow_idling) {
        /* Become idle if we can but are not draining */
        if (!activityRecorder.active() && !needToSignalDrained) {
            DPRINTF(Quiesce, "Suspending as the processor is idle\n");
            stop();
        }

        /* Deactivate all stages.  Note that the stages *could*
         *  activate and deactivate themselves but that's fraught
         *  with additional difficulty.
         *  As organised herre */
        activityRecorder.deactivateStage(Pipeline::CPUStageId);
        activityRecorder.deactivateStage(Pipeline::Fetch1StageId);
        activityRecorder.deactivateStage(Pipeline::Fetch2StageId);
        activityRecorder.deactivateStage(Pipeline::DecodeStageId1);
        activityRecorder.deactivateStage(Pipeline::DecodeStageId2);
        activityRecorder.deactivateStage(Pipeline::DecodeStageId3);
        activityRecorder.deactivateStage(Pipeline::DecodeStageId4);
        activityRecorder.deactivateStage(Pipeline::DecodeStageId5);
        activityRecorder.deactivateStage(Pipeline::DecodeStageId6);
        activityRecorder.deactivateStage(Pipeline::DecodeStageId7);
        activityRecorder.deactivateStage(Pipeline::DecodeStageId8);
        activityRecorder.deactivateStage(Pipeline::DecodeStageId9);
        activityRecorder.deactivateStage(Pipeline::DecodeStageId10);
        activityRecorder.deactivateStage(Pipeline::DecodeStageId11);
        activityRecorder.deactivateStage(Pipeline::DecodeStageId12);
        activityRecorder.deactivateStage(Pipeline::DecodeStageId13);
        activityRecorder.deactivateStage(Pipeline::DecodeStageId14);
        activityRecorder.deactivateStage(Pipeline::DecodeStageId15);
        activityRecorder.deactivateStage(Pipeline::DecodeStageId16);
        activityRecorder.deactivateStage(Pipeline::ExecuteStageId);
    }

    if (needToSignalDrained) /* Must be draining */
    {
        DPRINTF(Drain, "Still draining\n");
        if (isDrained()) {
            DPRINTF(Drain, "Signalling end of draining\n");
            cpu.signalDrainDone();
            needToSignalDrained = false;
            stop();
        }
    }
}
#else
void
Pipeline::evaluate()
{
    /** We tick the CPU to update the BaseCPU cycle counters */
    cpu.tick();

    /* Note that it's important to evaluate the stages in order to allow
     *  'immediate', 0-time-offset TimeBuffer activity to be visible from
     *  later stages to earlier ones in the same cycle */
    execute.evaluate();
    decode.evaluate();
    fetch2.evaluate();
    fetch1.evaluate();

    if (debug::MinorTrace)
        minorTrace();

    /* Update the time buffers after the stages */
    f1ToF2.evaluate();
    f2ToF1.evaluate();
    f2ToD.evaluate();
    dToE.evaluate();
    eToF1.evaluate();

    /* The activity recorder must be be called after all the stages and
     *  before the idler (which acts on the advice of the activity recorder */
    activityRecorder.evaluate();

    if (allow_idling) {
        /* Become idle if we can but are not draining */
        if (!activityRecorder.active() && !needToSignalDrained) {
            DPRINTF(Quiesce, "Suspending as the processor is idle\n");
            stop();
        }

        /* Deactivate all stages.  Note that the stages *could*
         *  activate and deactivate themselves but that's fraught
         *  with additional difficulty.
         *  As organised herre */
        activityRecorder.deactivateStage(Pipeline::CPUStageId);
        activityRecorder.deactivateStage(Pipeline::Fetch1StageId);
        activityRecorder.deactivateStage(Pipeline::Fetch2StageId);
        activityRecorder.deactivateStage(Pipeline::DecodeStageId);
        activityRecorder.deactivateStage(Pipeline::ExecuteStageId);
    }

    if (needToSignalDrained) /* Must be draining */
    {
        DPRINTF(Drain, "Still draining\n");
        if (isDrained()) {
            DPRINTF(Drain, "Signalling end of draining\n");
            cpu.signalDrainDone();
            needToSignalDrained = false;
            stop();
        }
    }
}
#endif

MinorCPU::MinorCPUPort &
Pipeline::getInstPort()
{
    return fetch1.getIcachePort();
}

MinorCPU::MinorCPUPort &
Pipeline::getDataPort()
{
    return execute.getDcachePort();
}

void
Pipeline::wakeupFetch(ThreadID tid)
{
    fetch1.wakeupFetch(tid);
}

bool
Pipeline::drain()
{
    DPRINTF(MinorCPU, "Draining pipeline by halting inst fetches. "
        " Execution should drain naturally\n");

    execute.drain();

    /* Make sure that needToSignalDrained isn't accidentally set if we
     *  are 'pre-drained' */
    bool drained = isDrained();
    needToSignalDrained = !drained;

    return drained;
}

void
Pipeline::drainResume()
{
    DPRINTF(Drain, "Drain resume\n");

    for (ThreadID tid = 0; tid < cpu.numThreads; tid++) {
        fetch1.wakeupFetch(tid);
    }

    execute.drainResume();
}

#if ENABLE_16STAGES
bool
Pipeline::isDrained()
{
    bool fetch1_drained = fetch1.isDrained();
    bool fetch2_drained = fetch2.isDrained();
    bool decode_drained = decode.isDrained();
    bool decode2_drained = decode2.isDrained();
    bool decode3_drained = decode3.isDrained();
    bool decode4_drained = decode4.isDrained();
    bool decode5_drained = decode5.isDrained();
    bool decode6_drained = decode6.isDrained();
    bool decode7_drained = decode7.isDrained();
    bool decode8_drained = decode8.isDrained();
    bool decode9_drained = decode9.isDrained();
    bool decode10_drained = decode10.isDrained();
    bool decode11_drained = decode11.isDrained();
    bool decode12_drained = decode12.isDrained();
    bool decode13_drained = decode13.isDrained();
    bool decode14_drained = decode14.isDrained();
    bool decode15_drained = decode15.isDrained();
    bool decode16_drained = decode16.isDrained();
    bool execute_drained = execute.isDrained();

    bool f1_to_f2_drained = f1ToF2.empty();
    bool f2_to_f1_drained = f2ToF1.empty();
    bool f2_to_d_drained = f2ToD.empty();
    bool d_to_d2_drained = dToD2.empty();
    bool d2_to_d3_drained = d2ToD3.empty();
    bool d3_to_d4_drained = d3ToD4.empty();
    bool d4_to_d5_drained = d4ToD5.empty();
    bool d5_to_d6_drained = d5ToD6.empty();
    bool d6_to_d7_drained = d6ToD7.empty();
    bool d7_to_d8_drained = d7ToD8.empty();
    bool d8_to_d9_drained = d8ToD9.empty();
    bool d9_to_d10_drained = d9ToD10.empty();
    bool d10_to_d11_drained = d10ToD11.empty();
    bool d11_to_d12_drained = d11ToD12.empty();
    bool d12_to_d13_drained = d12ToD13.empty();
    bool d13_to_d14_drained = d13ToD14.empty();
    bool d14_to_d15_drained = d14ToD15.empty();
    bool d15_to_d16_drained = d15ToD16.empty();
    bool d16_to_e_drained = d16ToE.empty();

    bool ret = fetch1_drained && fetch2_drained &&
        decode_drained && execute_drained &&
        decode2_drained && d_to_d2_drained &&
        decode3_drained && d2_to_d3_drained &&
        decode4_drained && d3_to_d4_drained &&
        decode5_drained && d4_to_d5_drained &&
        decode6_drained && d5_to_d6_drained &&
        decode7_drained && d6_to_d7_drained &&
        decode8_drained && d7_to_d8_drained &&
        decode9_drained && d8_to_d9_drained &&
        decode10_drained && d9_to_d10_drained &&
        decode11_drained && d10_to_d11_drained &&
        decode12_drained && d11_to_d12_drained &&
        decode13_drained && d12_to_d13_drained &&
        decode14_drained && d13_to_d14_drained &&
        decode15_drained && d14_to_d15_drained &&
        decode16_drained && d15_to_d16_drained &&
        f1_to_f2_drained && f2_to_f1_drained &&
        f2_to_d_drained && d16_to_e_drained;

    DPRINTF(MinorCPU, "Pipeline undrained stages state:%s%s%s%s%s%s%s%s\n",
        (fetch1_drained ? "" : " Fetch1"),
        (fetch2_drained ? "" : " Fetch2"),
        (decode_drained ? "" : " Decode"),
        (decode2_drained ? "" : " Decode2"),
        (decode3_drained ? "" : " Decode3"),
        (decode4_drained ? "" : " Decode4"),
        (decode5_drained ? "" : " Decode5"),
        (decode6_drained ? "" : " Decode6"),
        (decode7_drained ? "" : " Decode7"),
        (decode8_drained ? "" : " Decode8"),
        (decode9_drained ? "" : " Decode9"),
        (decode10_drained ? "" : " Decode10"),
        (decode11_drained ? "" : " Decode11"),
        (decode12_drained ? "" : " Decode12"),
        (decode13_drained ? "" : " Decode13"),
        (decode14_drained ? "" : " Decode14"),
        (decode15_drained ? "" : " Decode15"),
        (decode16_drained ? "" : " Decode16"),
        (execute_drained ? "" : " Execute"),
        (f1_to_f2_drained ? "" : " F1->F2"),
        (f2_to_f1_drained ? "" : " F2->F1"),
        (f2_to_d_drained ? "" : " F2->D"),
        (d_to_d2_drained ? "" : " D->D2"),
        (d2_to_d3_drained ? "" : " D2->D3"),
        (d3_to_d4_drained  ? "" : " D3->D4"),
        (d4_to_d5_drained ? "" : " D4->D5"),
        (d5_to_d6_drained ? "" : " D5->D6"),
        (d6_to_d7_drained ? "" : " D6->D7"),
        (d7_to_d8_drained ? "" : " D7->D8"),
        (d8_to_d9_drained ? "" : " D8->D9"),
        (d9_to_d10_drained ? "" : " D9->D10"),
        (d10_to_d11_drained ? "" : " D10->D11"),
        (d11_to_d12_drained ? "" : " D11->D12"),
        (d12_to_d13_drained ? "" : " D12->D13"),
        (d13_to_d14_drained ? "" : " D13->D14"),
        (d14_to_d15_drained ? "" : " D14->D15"),
        (d15_to_d16_drained ? "" : " D15->D16"),
        (d16_to_e_drained ? "" : " D16->E")
        );

    return ret;
}
#else
bool
Pipeline::isDrained()
{
    bool fetch1_drained = fetch1.isDrained();
    bool fetch2_drained = fetch2.isDrained();
    bool decode_drained = decode.isDrained();
    bool execute_drained = execute.isDrained();

    bool f1_to_f2_drained = f1ToF2.empty();
    bool f2_to_f1_drained = f2ToF1.empty();
    bool f2_to_d_drained = f2ToD.empty();
    bool d_to_e_drained = dToE.empty();

    bool ret = fetch1_drained && fetch2_drained &&
        decode_drained && execute_drained &&
        f1_to_f2_drained && f2_to_f1_drained &&
        f2_to_d_drained && d_to_e_drained;

    DPRINTF(MinorCPU, "Pipeline undrained stages state:%s%s%s%s%s%s%s%s\n",
        (fetch1_drained ? "" : " Fetch1"),
        (fetch2_drained ? "" : " Fetch2"),
        (decode_drained ? "" : " Decode"),
        (execute_drained ? "" : " Execute"),
        (f1_to_f2_drained ? "" : " F1->F2"),
        (f2_to_f1_drained ? "" : " F2->F1"),
        (f2_to_d_drained ? "" : " F2->D"),
        (d_to_e_drained ? "" : " D->E")
        );

    return ret;
}
#endif

} // namespace minor
} // namespace gem5
