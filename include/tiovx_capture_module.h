/*
 *
 * Copyright (c) 2023 Texas Instruments Incorporated
 *
 * All rights reserved not granted herein.
 *
 * Limited License.
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive
 * license under copyrights and patents it now or hereafter owns or controls to make,
 * have made, use, import, offer to sell and sell ("Utilize") this software subject to the
 * terms herein.  With respect to the foregoing patent license, such license is granted
 * solely to the extent that any such patent is necessary to Utilize the software alone.
 * The patent license shall not apply to any combinations which include this software,
 * other than combinations with devices manufactured by or for TI ("TI Devices").
 * No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce this license
 * (including the above copyright notice and the disclaimer and (if applicable) source
 * code license limitations below) in the documentation and/or other materials provided
 * with the distribution
 *
 * Redistribution and use in binary form, without modification, are permitted provided
 * that the following conditions are met:
 *
 * *       No reverse engineering, decompilation, or disassembly of this software is
 * permitted with respect to any software provided in binary form.
 *
 * *       any redistribution and use are licensed by TI for use only with TI Devices.
 *
 * *       Nothing shall obligate TI to provide you with source code for the software
 * licensed and provided to you in object code.
 *
 * If software source code is provided to you, modification and redistribution of the
 * source code are permitted provided that the following conditions are met:
 *
 * *       any redistribution and use of the source code, including any resulting derivative
 * works, are licensed by TI for use only with TI Devices.
 *
 * *       any redistribution and use of any object code compiled from the source code
 * and any resulting derivative works, are licensed by TI for use only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its suppliers
 *
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
 #ifndef _TIOVX_CAPTURE_MODULE
 #define _TIOVX_CAPTURE_MODULE

#include <tiovx_modules_common.h>
#include <TI/video_io_capture.h>
#include <tiovx_sensor_module.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /*! Capture node object */
    vx_node node;

    /*! Capture node user data object for configuration of node */
    vx_user_data_object config;

    /*! Capture node params structure to initialize config object */
    tivx_capture_params_t params;

    /*! Capture node object array output */
    vx_object_array image_arr[TIOVX_MODULES_MAX_BUFQ_DEPTH];

    /*! Bufq depth of output image*/
    vx_int32 out_bufq_depth;

    /*! Capture node format, taken from sensor_out_format */
    /*! sensor_out_format : 0=RAW, 1=YUV */
    vx_uint32 capture_format;

    /*! Capture node graph parameter index */
    vx_int32 graph_parameter_index;

    /*! Name of capture module */
    vx_char objName[TIOVX_MODULES_MAX_OBJ_NAME_SIZE];

    /*! Raw image used when camera gets disconnected */
    tivx_raw_image error_frame_raw_image;

    /*! Flag to enable detection of camera disconnection */
    vx_uint8 enable_error_detection;

    /*! Flag to indicate whether or not the intermediate output is written */
    vx_int32 en_out_image_write;

    /* These params are needed only for writing intermediate output */
    vx_array file_path;
    vx_array file_prefix;
    vx_node write_node;
    vx_user_data_object write_cmd;

    vx_char output_file_path[TIVX_FILEIO_FILE_PATH_LENGTH];

} TIOVXCaptureModuleObj;

/** \brief Capture module init helper function
 *
 * This capture init helper function will create all the data objects required to create the capture
 * node
 *
 * \param [in]  context    OpenVX context which must be created using \ref vxCreateContext
 * \param [out] captureObj Capture Module object which gets populated with capture node data objects
 * \param [in]  sensorObj  Sensor Module object used to initialize capture data object parameters;
 *                         must be initialized prior to passing to this function
 *
 */
vx_status tiovx_capture_module_init(vx_context context, TIOVXCaptureModuleObj *obj, SensorObj *sensorObj);

/** \brief Capture module deinit helper function
 *
 * This capture deinit helper function will release all the data objects created during the \ref tiovx_capture_module_init call
 *
 * \param [in,out] captureObj    Capture Module object which contains capture node data objects which are released in this function
 * 
 */
vx_status tiovx_capture_module_deinit(TIOVXCaptureModuleObj *obj);

/** \brief Capture module delete helper function
 *
 * This capture delete helper function will delete the capture node and write node that is created during the \ref tiovx_capture_module_create call
 *
 * \param [in,out] captureObj   Capture Module object which contains capture node objects which are released in this function
 *
 */
vx_status tiovx_capture_module_delete(TIOVXCaptureModuleObj *obj);

/** \brief Capture module create helper function
 *
 * This capture create helper function will create the node using all the data objects created during the \ref tiovx_capture_module_init call.
 * Internally calls \ref tiovx_capture_module_add_write_output_node if en_out_image_write is set
 *
 * \param [in]     graph       OpenVX graph that has been created using \ref vxCreateGraph and where the capture node is created
 * \param [in,out] captureObj  Capture Module object which contains capture node and write node which are created in this function
 *
 */
vx_status tiovx_capture_module_create(vx_graph graph, TIOVXCaptureModuleObj *obj, const char* target_string);

/** \brief Capture module release buffers helper function
 *
 * This Capture helper function will release the buffers alloted during vxVerifyGraph stage
 *
 * \param [in] obj  Capture Module object
 *
 */
vx_status tiovx_capture_module_release_buffers(TIOVXCaptureModuleObj *obj);

/** \brief Capture module write output helper function
 *
 * This capture create helper function will create the node for writing the capture output
 *
 * \param [in]     graph       OpenVX graph
 * \param [in,out] captureObj  Capture Module object which contains capture node and write node which are created in this function
 *
 */
vx_status tiovx_capture_module_add_write_output_node(vx_graph graph, TIOVXCaptureModuleObj *obj);

/** \brief Capture module write output helper function
 *
 * This capture create helper function will create the node for writing the capture output
 *
 * \param [in] captureObj    Capture Module object which contains the write node used in this function
 * \param [in] start_frame   Starting frame to write
 * \param [in] num_frames    Total number of frames to write
 * \param [in] num_skip      Number of capture frames to skip writing
 *
 */
vx_status tiovx_capture_module_send_write_output_cmd(TIOVXCaptureModuleObj *obj, vx_uint32 start_frame, vx_uint32 num_frames, vx_uint32 num_skip);

/** \brief Capture module module params initialization
 *
 * This capture create helper function will initialize some parameters of capture node by using details
 * from sensorObj
 *
 * \param [in] captureObj    Capture Module object which contains the write node used in this function
 * \param [in] sensorObj     Sensor Module object used to initialize capture data object parameters;
 *                           must be initialized prior to passing to this function
 * 
 */
void tiovx_capture_module_params_init(TIOVXCaptureModuleObj *captureObj, SensorObj *sensorObj);

/** \brief Capture module send error frmae
 *
 * This capture helper function send the blank frame to the capture node over a control command.  This frame is
 * used as the output for when a camera is disconnected
 *
 * \param [in] captureObj   Capture Module object which contains the error frame object for using as blank frame
 *
 */
vx_status tiovx_capture_module_send_error_frame(TIOVXCaptureModuleObj *captureObj);

#ifdef __cplusplus
}
#endif

#endif // _TIOVX_CAPTURE_MODULE
