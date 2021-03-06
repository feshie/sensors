fndef _PB_BUFFER_PB_H_
#define _PB_BUFFER_PB_H_
#include "pb.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
typedef enum _Rs485_Type {
    Rs485_Type_READ = 0,
    Rs485_Type_DATA = 1
} Rs485_Type;

typedef enum _Rs485_Sensor {
    Rs485_Sensor_OW = 0,
    Rs485_Sensor_TA_CHAIN = 1,
    Rs485_Sensor_ADC0 = 2,
    Rs485_Sensor_ADC1 = 3,
    Rs485_Sensor_ADC2 = 4,
    Rs485_Sensor_ADC3 = 5
} Rs485_Sensor;

/* Struct definitions */
typedef struct _AnalogData {
    int32_t adc;
    float value;
} AnalogData;

typedef struct _OwData {
    int32_t id;
    float value;
} OwData;

typedef struct _TempAccelData {
    int32_t id;
    float pitch;
    float roll;
    float temp;
} TempAccelData;

typedef struct _Rs485 {
    int32_t dst;
    Rs485_Type type;
    bool has_sensor;
    Rs485_Sensor sensor;
    size_t ow_count;
    OwData ow[5];
    size_t ad_count;
    AnalogData ad[4];
    size_t tad_count;
    TempAccelData tad[4];
} Rs485;

/* Default values for struct fields */
extern const Rs485_Type Rs485_type_default;

/* Field tags (for use in manual encoding/decoding) */
#define AnalogData_adc_tag                       1
#define AnalogData_value_tag                     2
#define OwData_id_tag                            1
#define OwData_value_tag                         2
#define TempAccelData_id_tag                     1
#define TempAccelData_pitch_tag                  2
#define TempAccelData_roll_tag                   3
#define TempAccelData_temp_tag                   4
#define Rs485_dst_tag                            1
#define Rs485_type_tag                           2
#define Rs485_sensor_tag                         3
#define Rs485_ow_tag                             4
#define Rs485_ad_tag                             5
#define Rs485_tad_tag                            6

/* Struct field encoding specification for nanopb */
extern const pb_field_t Rs485_fields[7];
extern const pb_field_t OwData_fields[3];
extern const pb_field_t AnalogData_fields[3];
extern const pb_field_t TempAccelData_fields[5];

/* Maximum encoded size of messages (where known) */
#define Rs485_size                               297
#define OwData_size                              16
#define AnalogData_size                          16
#define TempAccelData_size                       26

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif

