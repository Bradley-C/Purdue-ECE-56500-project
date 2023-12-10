
from m5.SimObject import SimObject
from m5.params import *
from m5.proxy import *


class ConstantVerificationUnit(SimObject):
    type = 'ConstantVerificationUnit'
    cxx_class = 'gem5::load_value_prediction::ConstantVUnit'
    cxx_header = "cpu/lvpu/cvu2.hh"
    numThreads = Param.Unsigned(1, "Number of threads")
    CVUEntries = Param.Unsigned(1024, "Number of LVPT entries")
    numAddrHold = Param.Unsigned(1, "Number of data address per entry")
    CVUTagSize = Param.Unsigned(32, "Size of the CVU tags, in bits")
    instShiftAmt = Param.Unsigned(64-10, "Number of bits to shift ints by")

