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

#ifndef _TIOVX_TIDL_MODULE
#define _TIOVX_TIDL_MODULE

/**
 * \defgroup group_tiovx_modules_tidl TIDL Node Module
 *
 * \brief This section contains module APIs for the TIOVX TIDL node tivxTIDLNode
 *
 * \ingroup group_tiovx_modules
 *
 * @{
 */

#include "tiovx_modules_common.h"
#include "itidl_ti.h"

#ifdef __cplusplus
extern "C" {
#endif

// #define COMPUTE_CHECKSUM

typedef struct {
  /*! TIDL node object */
  vx_node    node;

  /*! TIDL kernel object */
  vx_kernel  kernel;

  /*! TIDL config user data object */
  vx_user_data_object  config;
  tivxTIDLJ7Params params;

  /*! TIDL network user data object */
  vx_user_data_object  network;

  /*! TIDL create params user data object */
  vx_user_data_object  createParams;

  /*! TIDL object array of input args */
  vx_object_array  in_args_arr;

  /*! TIDL object array of output args */
  vx_object_array  out_args_arr;

  /*! TIDL object array of trace data */
  vx_object_array trace_data_arr;

  /*! TIDL number of input tensors */
  vx_uint32 num_input_tensors;

  /*! TIDL number of output tensors */
  vx_uint32 num_output_tensors;

  /*! TIDL graph parameter index */
  vx_int32 graph_parameter_index;

  /*! Config structure file path */
  vx_char* config_file_path;

  /*! Network structure file path */
  vx_char* network_file_path;

  /*! Name of TIDL module */
  vx_char objName[TIOVX_MODULES_MAX_OBJ_NAME_SIZE];

  /*! Config structure checksum */
  vx_uint8 config_checksum[TIVX_TIDL_J7_CHECKSUM_SIZE];

  /*! Network structure checksum */
  vx_uint8 network_checksum[TIVX_TIDL_J7_CHECKSUM_SIZE];

  /*! TIDL input tensor Object  */
  TensorObj input[TIOVX_MODULES_MAX_TENSORS];
  
  /*! TIDL output tensor Object  */
  TensorObj output[TIOVX_MODULES_MAX_TENSORS];

  vx_int32 num_cameras;

  /*! Flag to enable writing TIDL output  */
  vx_int32 en_out_tensor_write;
  
  /*! Node used to write TIDL output */
  vx_node write_node;
  
  /*! File path used to write TIDL node output */
  vx_array file_path;
  
  /*! File path prefix used to write TIDL node output */
  vx_array file_prefix;
  
  /*! User data object containing write cmd parameters */
  vx_user_data_object write_cmd;
  
  /*! Output file path for TIDL node output */
  vx_char output_file_path[TIVX_FILEIO_FILE_PATH_LENGTH];

} TIOVXTIDLModuleObj;

/** \brief TIDL module init helper function
 *
 * This TIDL init helper function will create all the data objects required to create the TIDL
 * node
 *
 * \param [in]  context     OpenVX context which must be created using \ref vxCreateContext
 * \param [out] tidlObj     TIDL Module object which gets populated with TIDL node data objects
 * \param [in]  objName     String of the name of this object
 * \param [in]  num_cameras Number of cameras used by TIDL
 *
 */
vx_status tiovx_tidl_module_init(vx_context context, TIOVXTIDLModuleObj *tidlObj, char *objName);

/** \brief TIDL module deinit helper function
 *
 * This TIDL deinit helper function will release all the data objects created during the \ref tiovx_tidl_module_init call
 *
 * \param [in,out] tidlObj  TIDL Module object which contains TIDL node data objects which are released in this function
 *
 */
vx_status tiovx_tidl_module_deinit(TIOVXTIDLModuleObj *tidlObj);

/** \brief TIDL module delete helper function
 *
 * This TIDL delete helper function will delete the TIDL node that is created during the \ref app_create_graph_tidl call
 *
 * \param [in,out] tidlObj  TIDL Module object which contains TIDL node objects which are released in this function
 *
 */
vx_status tiovx_tidl_module_delete(TIOVXTIDLModuleObj *tidlObj);

/** \brief TIDL module create helper function
 *
 * This TIDL create helper function will create the node using all the data objects created during the \ref app_init_tidl call.
 *
 * \param [in]     context           OpenVX context which must be created using \ref vxCreateContext
 * \param [in]     graph             OpenVX graph that has been created using \ref vxCreateGraph and where the TIDL node is created
 * \param [in,out] tidlObj           TIDL Module object which contains TIDL node which is created in this function
 * \param [in,out] input_tensor_arr  Input tensors to TIDL node; must be created separately outside the TIDL module
 *
 */
vx_status tiovx_tidl_module_create(vx_context context, vx_graph graph, TIOVXTIDLModuleObj *tidlObj, vx_object_array input_tensor_arr[]);

/** \brief TIDL module write output helper function
 *
 * This TIDL helper function will create the node for writing the TIDL output
 *
 * \param [in]     graph    OpenVX graph
 * \param [in,out] tidlObj  TIDL Module object which contains TIDL node and write node which are created in this function
 *
 */
vx_status tiovx_tidl_module_add_write_output_node(vx_graph graph, TIOVXTIDLModuleObj *tidlObj);

/** \brief TIDL module send write output command function
 *
 * This TIDL helper function will send commands to write output to a file.
 *
 * \param [in] tidlObj       TIDL Module object which contains the write node used in this function
 * \param [in] start_frame   Starting frame to write
 * \param [in] num_frames    Total number of frames to write
 * \param [in] num_skip      Number of capture frames to skip writing
 *
 */
vx_status tiovx_tidl_module_send_write_output_cmd(TIOVXTIDLModuleObj *tidlObj, vx_uint32 start_frame, vx_uint32 num_frames, vx_uint32 num_skip);

/* @} */

#ifdef __cplusplus
}
#endif

#endif 