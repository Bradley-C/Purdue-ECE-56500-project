
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

    instShiftAmt = Param.Unsigned(1, "Number of bits to shift instructions by")
    # lct = Param.LoadClassficationTable(LoadClassificationTable(),
    #                                    "Load Classification Table")
    # cvu = Param.ConstantVerificationUnit(ConstantVerificationUnit(),
    #                                      "Constant Verification Unit")
    # lvpt = Param.LoadValuePredictionTable(LoadValuePredictionTable(),
    #                                      "Load Value Prediction Table")
