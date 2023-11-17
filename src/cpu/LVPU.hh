#include "base/named.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(Minor, minor);
class LVPU : public Named
{


    public:
        unsigned int lvpt_entry_length
        unsigned int lct_entry_length
        
        class LVPT : public Named
        {

            // 

            protected:
                /** My owner */
                LVPU &lvpu;
            public:
                std::array <lvpt_entry_length,1>   TableEntry={}
        
                void updateEntry();
        }


        }

        class LCT : public Named
        {
            // 
            protected:
                /** My owner */
                LVPU &lvpu;
            public:
                    std::array <unsigned int, lct_entry_length>   SaturatingBitCount={}
        


        }

        class CVU : public Named
        {
            // 
            protected:
                /** My owner */
                LVPU &lvpu;



        }
}