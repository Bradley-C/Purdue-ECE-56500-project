# Copyright (c) 2012 Mark D. Hill and David A. Wood
# Copyright (c) 2015 The University of Wisconsin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from m5.SimObject import SimObject
from m5.params import *
from m5.proxy import *

# class LoadClassificationTable(SimObject):
#     type = 'LoadClassificationTable'
#     cxx_class = 'gem5::load_value_prediction::LoadClassificationTable'
#     cxx_header = "cpu/lvpu/lct.hh"

#     lctSize = Param.Unsigned(256, "Size of load classification table")
#     lctCtrlBits = Param.Unsigned(2, "Bits per counter")


class ConstantVerificationUnit(SimObject):
    type = 'ConstantVerificationUnit'
    cxx_class = 'gem5::load_value_prediction::ConstantVerificationUnit'
    cxx_header = "cpu/lvpu/cvu.hh"
    CVUEntries = Param.Unsigned(1024, "Number of LVPT entries")
    CVUTagSize = Param.Unsigned(32, "Size of the CVU tags, in bits")

class LoadValuePredictionTable(SimObject):
    type = 'LoadValuePredictionTable'
    cxx_class = 'gem5::load_value_prediction::LoadValuePredictionTable'
    cxx_header = "cpu/lvpu/lvpt.hh"

    LVPTEntries = Param.Unsigned(1024, "Number of LVPT entries")
    LVPTTagSize = Param.Unsigned(22, "Size of the LVPT tags, in bits")

class LoadValuePredictor(SimObject):
    type = 'LoadValuePredictor'
    cxx_class = 'gem5::load_value_prediction::LVPUnit'
    cxx_header = "cpu/lvpu/lvpu.hh"

    numThreads = Param.Unsigned(Parent.numThreads, "Number of threads")
    instShiftAmt = Param.Unsigned(2, "Number of bits to shift instructions by")
    # lct = Param.LoadClassficationTable(LoadClassificationTable(),
    #                                    "Load Classification Table")
    cvu = Param.ConstantVerificationUnit(ConstantVerificationUnit(),
                                          "Constant Verification Unit")
    lvpt = Param.LoadValuePredictionTable(LoadValuePredictionTable(),
                                          "Load Value Prediction Table")