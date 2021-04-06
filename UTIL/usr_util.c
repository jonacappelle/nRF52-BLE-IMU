#include "usr_util.h"

// void float_to_uint8_t( float *input, uint8_t *out )
// {
//     union{
//         float in;
//         uint8_t bytes[sizeof(float)];
//     }thing;
//  
//     thing.in = input[0];
//  
//     for( int i=0; i<sizeof(float); i++)
//     {
//         out[i] = thing.bytes[i];
//     }
// }
// 
//void uint8_t_to_float( uint8_t *input, float *out )
// {
//     union{
//         float in;
//         uint8_t bytes[sizeof(float)];
//     }thing;
//  
//			for( int i=0; i<sizeof(float); i++)
//     {
//         thing.bytes[i] = input[i];
//     }
//		 
//     out[0] = thing.in;
//  

// }