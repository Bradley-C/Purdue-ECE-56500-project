
from m5.SimObject import SimObject
from m5.params import *
from m5.proxy import *


class LoadValuePredictor(SimObject):
    type = 'LoadValuePredictor'
    cxx_class = 'gem5::load_value_prediction::LVPredUnit'
    cxx_header = "cpu/lvpu/lvpu.hh"

    numThreads = Param.Unsigned(1, "Number of threads")
    LVPTEntries = Param.Unsigned(1024, "Number of LVPT entries")
    lctSize = Param.Unsigned(512, "Number of LCT entries")
    lctBits = Param.Unsigned(2, "Number of bits per LCT entry")

    instShiftAmt = Param.Unsigned(3, "Number of bits to shift instructions by")
    # lct = Param.LoadClassficationTable(LoadClassificationTable(),
    #                                    "Load Classification Table")
    # cvu = Param.ConstantVerificationUnit(ConstantVerificationUnit(),
    #                                      "Constant Verification Unit")
    # lvpt = Param.LoadValuePredictionTable(LoadValuePredictionTable(),
    #                                      "Load Value Prediction Table")


class ConstantVerificationUnit(SimObject):
    type = 'ConstantVerificationUnit'
    cxx_class = 'gem5::load_value_prediction::ConstantVUnit'
    cxx_header = "cpu/lvpu/cvu2.hh"
    numThreads = Param.Unsigned(Parent.numThreads, "Number of threads")
    CVUEntries = Param.Unsigned(1024, "Number of CVU entries")
    numAddrHold = Param.Unsigned(1, "Number of data address per entry")
    CVUTagSize = Param.Unsigned(32, "Size of the CVU tags, in bits")
    instShiftAmt = Param.Unsigned(64-10, "Number of bits to shift ints by")
