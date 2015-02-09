/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.2.9-dev at Sun Aug 10 19:48:45 2014. */

#include "buffer.h"

const Rs485_Type Rs485_type_default = Rs485_Type_READ;


const pb_field_t Rs485_fields[7] = {
    PB_FIELD2(  1, INT32   , REQUIRED, STATIC  , FIRST, Rs485, dst, dst, 0),
    PB_FIELD2(  2, ENUM    , REQUIRED, STATIC  , OTHER, Rs485, type, dst, &Rs485_type_default),
    PB_FIELD2(  3, ENUM    , OPTIONAL, STATIC  , OTHER, Rs485, sensor, type, 0),
    PB_FIELD2(  4, MESSAGE , REPEATED, STATIC  , OTHER, Rs485, ow, sensor, &OwData_fields),
    PB_FIELD2(  5, MESSAGE , REPEATED, STATIC  , OTHER, Rs485, ad, ow, &AnalogData_fields),
    PB_FIELD2(  6, MESSAGE , REPEATED, STATIC  , OTHER, Rs485, tad, ad, &TempAccelData_fields),
    PB_LAST_FIELD
};

const pb_field_t OwData_fields[3] = {
    PB_FIELD2(  1, INT32   , REQUIRED, STATIC  , FIRST, OwData, id, id, 0),
    PB_FIELD2(  2, FLOAT   , REQUIRED, STATIC  , OTHER, OwData, value, id, 0),
    PB_LAST_FIELD
};

const pb_field_t AnalogData_fields[3] = {
    PB_FIELD2(  1, INT32   , REQUIRED, STATIC  , FIRST, AnalogData, adc, adc, 0),
    PB_FIELD2(  2, FLOAT   , REQUIRED, STATIC  , OTHER, AnalogData, value, adc, 0),
    PB_LAST_FIELD
};

const pb_field_t TempAccelData_fields[5] = {
    PB_FIELD2(  1, INT32   , REQUIRED, STATIC  , FIRST, TempAccelData, id, id, 0),
    PB_FIELD2(  2, FLOAT   , REQUIRED, STATIC  , OTHER, TempAccelData, pitch, id, 0),
    PB_FIELD2(  3, FLOAT   , REQUIRED, STATIC  , OTHER, TempAccelData, roll, pitch, 0),
    PB_FIELD2(  4, FLOAT   , REQUIRED, STATIC  , OTHER, TempAccelData, temp, roll, 0),
    PB_LAST_FIELD
};


/* Check that field information fits in pb_field_t */
#if !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_32BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in 8 or 16 bit
 * field descriptors.
 */
STATIC_ASSERT((pb_membersize(Rs485, ow[0]) < 65536 && pb_membersize(Rs485, ad[0]) < 65536 && pb_membersize(Rs485, tad[0]) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_Rs485_OwData_AnalogData_TempAccelData)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_16BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in the default
 * 8 bit descriptors.
 */
STATIC_ASSERT((pb_membersize(Rs485, ow[0]) < 256 && pb_membersize(Rs485, ad[0]) < 256 && pb_membersize(Rs485, tad[0]) < 256), YOU_MUST_DEFINE_PB_FIELD_16BIT_FOR_MESSAGES_Rs485_OwData_AnalogData_TempAccelData)
#endif



