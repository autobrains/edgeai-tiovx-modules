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

#include <tiovx_tidl_module.h>

static vx_status tiovx_tidl_module_create_config(vx_context context, TIOVXTIDLModuleObj *obj);
static vx_status tiovx_tidl_module_create_network(vx_context context, TIOVXTIDLModuleObj *obj);
static vx_status tiovx_tidl_module_update_checksums(TIOVXTIDLModuleObj *obj);
static vx_status tiovx_tidl_module_set_createParams(vx_context context, TIOVXTIDLModuleObj *obj);
static vx_status tiovx_tidl_module_create_inArgs(vx_context context, TIOVXTIDLModuleObj *obj);
static vx_status tiovx_tidl_module_create_outArgs(vx_context context, TIOVXTIDLModuleObj *obj);
static vx_status tiovx_tidl_module_create_inputs(vx_context context, TIOVXTIDLModuleObj *obj);
static vx_status tiovx_tidl_module_create_outputs(vx_context context, TIOVXTIDLModuleObj *obj);
static void initParam(vx_reference params[], uint32_t _max_params);
static void addParam(vx_reference params[], vx_reference obj);
#ifdef COMPUTE_CHECKSUM
static void getQC(uint8_t *pIn, uint8_t *pOut, int32_t inSize);
#endif

static uint32_t num_params;
static uint32_t max_params;

vx_status tiovx_tidl_module_init(vx_context context, TIOVXTIDLModuleObj *tidlObj, char *objName)
{
    vx_status status = VX_SUCCESS;

    tidlObj->kernel = NULL;

    status = tiovx_tidl_module_create_config(context, tidlObj);

    if(status == VX_SUCCESS)
    {
        status = tiovx_tidl_module_create_network(context, tidlObj);
    }    

    if(status == VX_SUCCESS)
    {
        status = tiovx_tidl_module_update_checksums(tidlObj);
    }

    if(status == VX_SUCCESS)
    {
        status = tiovx_tidl_module_set_createParams(context, tidlObj);
    }
    
    if(status == VX_SUCCESS)
    {
        status = tiovx_tidl_module_create_inArgs(context, tidlObj);
    }

    if(status == VX_SUCCESS)
    {
        status = tiovx_tidl_module_create_outArgs(context, tidlObj);
    }

    if(status == VX_SUCCESS)
    {
        status = tiovx_tidl_module_create_inputs(context, tidlObj);
    }

    if(status == VX_SUCCESS)
    {
        status = tiovx_tidl_module_create_outputs(context, tidlObj);
    }

    if(status == VX_SUCCESS)
    {
        tidlObj->kernel = tivxAddKernelTIDL(context, tidlObj->num_input_tensors, tidlObj->num_output_tensors);
        status = vxGetStatus((vx_reference)tidlObj->kernel);
    }

    snprintf(tidlObj->objName, TIOVX_MODULES_MAX_OBJ_NAME_SIZE, "%s", objName);

    return status;

}

vx_status tiovx_tidl_module_deinit(TIOVXTIDLModuleObj *tidlObj)
{
    vx_status status = VX_SUCCESS;
    vx_int32 q, i;

    TIOVX_MODULE_PRINTF("[TIDL-MODULE] Releasing config handle!\n");
    status = vxReleaseUserDataObject(&tidlObj->config);

    if(status == VX_SUCCESS)
    {
        TIOVX_MODULE_PRINTF("[TIDL-MODULE] Releasing network handle!\n");
        status = vxReleaseUserDataObject(&tidlObj->network);
    }

    if(status == VX_SUCCESS)
    {
        TIOVX_MODULE_PRINTF("[TIDL-MODULE] Releasing network handle!\n");
        status = vxReleaseUserDataObject(&tidlObj->createParams);
    }

    if(status == VX_SUCCESS)
    {
        TIOVX_MODULE_PRINTF("[TIDL-MODULE] Releasing object array of input args!\n");
        status = vxReleaseObjectArray(&tidlObj->in_args_arr);
    }

    if(status == VX_SUCCESS)
    {
        TIOVX_MODULE_PRINTF("[TIDL-MODULE] Releasing object array of output args!\n");
        vxReleaseObjectArray(&tidlObj->out_args_arr);
    }

    for(i = 0; i < tidlObj->num_input_tensors; i++)
    {
        for(q = 0; q < tidlObj->input[i].bufq_depth; q++)
        {
            if((vx_status)VX_SUCCESS == status)
            {
                TIOVX_MODULE_PRINTF("[TIDL-MODULE] Releasing input tensor handle %d, bufq %d!\n", i, q);
                status = vxReleaseTensor(&tidlObj->input[i].tensor_handle[q]);
            }
            if((vx_status)VX_SUCCESS == status)
            {
                TIOVX_MODULE_PRINTF("[TIDL-MODULE] Releasing input object array %d, bufq %d!\n", i, q);
                status = vxReleaseObjectArray(&tidlObj->input[i].arr[q]);
            }
        }
    }

    for(i = 0; i < tidlObj->num_output_tensors; i++)
    {
        for(q = 0; q < tidlObj->output[i].bufq_depth; q++)
        {
            if((vx_status)VX_SUCCESS == status)
            {
                TIOVX_MODULE_PRINTF("[TIDL-MODULE] Releasing output tensor handle %d, bufq %d!\n", i, q);
                status = vxReleaseTensor(&tidlObj->output[i].tensor_handle[q]);
            }
            if((vx_status)VX_SUCCESS == status)
            {
                TIOVX_MODULE_PRINTF("[TIDL-MODULE] Releasing output object array %d, bufq %d!\n", i, q);
                status = vxReleaseObjectArray(&tidlObj->output[i].arr[q]);
            }
        }
    }

    return status;
}

vx_status tiovx_tidl_module_delete(TIOVXTIDLModuleObj *tidlObj)
{
    vx_status status = VX_SUCCESS;

    if(tidlObj->node != NULL)
    {
        status = vxReleaseNode(&tidlObj->node);
    }
    if(tidlObj->kernel != NULL)
    {
        if(status == VX_SUCCESS)
            vxRemoveKernel(tidlObj->kernel);
    }

    return status;
}

vx_status tiovx_tidl_module_create(vx_context context, vx_graph graph, TIOVXTIDLModuleObj *tidlObj, 
                                    vx_object_array input_tensor_arr_user[])
{
    vx_status status = VX_SUCCESS;
    vx_reference params[TIOVX_MODULES_MAX_PARAMS];
    vx_object_array input_arr[TIOVX_MODULES_MAX_TENSORS];
    vx_tensor input_tensor[TIOVX_MODULES_MAX_TENSORS];
    vx_tensor output_tensor[TIOVX_MODULES_MAX_TENSORS];
    vx_int32 i;

    tidlObj->node = NULL;

    /* Initialize param array */
    initParam(params, TIOVX_MODULES_MAX_PARAMS);

    /* The 1st param MUST be config array */
    addParam(params, (vx_reference)tidlObj->config);

    /* The 2nd param MUST be network tensor */
    addParam(params, (vx_reference)tidlObj->network);

    /* The 3rd param MUST be create params */
    addParam(params, (vx_reference)tidlObj->createParams);

    /* The 4th param MUST be inArgs */
    vx_user_data_object inArgs = (vx_user_data_object)vxGetObjectArrayItem(tidlObj->in_args_arr, 0);
    addParam(params, (vx_reference)inArgs);
    vxReleaseUserDataObject(&inArgs);

    /* The 5th param MUST be outArgs */
    vx_user_data_object outArgs = (vx_user_data_object)vxGetObjectArrayItem(tidlObj->out_args_arr, 0);
    addParam(params, (vx_reference)outArgs);
    vxReleaseUserDataObject(&outArgs);

    /* The 6th param MUST be NULL if trace data dump is not enabled */
    addParam(params, NULL);

    for(i = 0; i < tidlObj->num_input_tensors; i++)
    {
        if(input_tensor_arr_user[i] == NULL)
        {
            input_arr[i] = tidlObj->input[i].arr[0];
        }
        else
        {
            input_arr[i] = input_tensor_arr_user[i];
        }
    }

    /* Create TIDL Node */
    for(i = 0; i < tidlObj->num_input_tensors; i++)
    {
        input_tensor[i]  = (vx_tensor)vxGetObjectArrayItem((vx_object_array)input_arr[i], 0);
    }

    for(i = 0; i < tidlObj->num_output_tensors; i++)
    {
        output_tensor[i] = (vx_tensor)vxGetObjectArrayItem((vx_object_array)tidlObj->output[i].arr[0], 0);
    }

    tidlObj->node = tivxTIDLNode(graph, tidlObj->kernel, params, input_tensor, output_tensor);
    status = vxGetStatus((vx_reference)tidlObj->node);
    vxSetReferenceName((vx_reference)tidlObj->node, "tidl_node");
    vxSetNodeTarget(tidlObj->node, VX_TARGET_STRING, TIVX_TARGET_DSP_C7_1);

    vx_bool replicate[16];
    replicate[TIVX_KERNEL_TIDL_IN_CONFIG_IDX] = vx_false_e;
    replicate[TIVX_KERNEL_TIDL_IN_NETWORK_IDX] = vx_false_e;
    replicate[TIVX_KERNEL_TIDL_IN_CREATE_PARAMS_IDX] = vx_false_e;
    replicate[TIVX_KERNEL_TIDL_IN_IN_ARGS_IDX] = vx_true_e;
    replicate[TIVX_KERNEL_TIDL_IN_OUT_ARGS_IDX] = vx_true_e;
    replicate[TIVX_KERNEL_TIDL_IN_TRACE_DATA_IDX] = vx_false_e;

    for(i = 0; i < tidlObj->num_input_tensors; i++)
    {
        replicate[TIVX_KERNEL_TIDL_NUM_BASE_PARAMETERS + i] = vx_true_e;
    }

    for(i = 0; i < tidlObj->num_output_tensors; i++)
    {
        replicate[TIVX_KERNEL_TIDL_NUM_BASE_PARAMETERS + tidlObj->num_input_tensors + i] = vx_true_e;
    }

    vxReplicateNode(graph, tidlObj->node, replicate, TIVX_KERNEL_TIDL_NUM_BASE_PARAMETERS + tidlObj->num_input_tensors + tidlObj->num_output_tensors);

    if((vx_status)VX_SUCCESS == status)
    {
        if(tidlObj->en_out_tensor_write == 1)
        {
            if(output_tensor[0] != NULL)
            {
                status = tiovx_tidl_module_add_write_output_node(graph, tidlObj);
                if(status != VX_SUCCESS)
                {
                    TIOVX_MODULE_ERROR("[TIDL-MODULE] Unable to create write node for output!\n");
                }
            }
        }
    }
    else
    {
        TIOVX_MODULE_ERROR("[TIDL-MODULE] Unable to create node! \n");
    }

    for(i = 0; i < tidlObj->num_input_tensors; i++)
    {
        vxReleaseTensor(&input_tensor[i]);
    }

    for(i = 0; i < tidlObj->num_output_tensors; i++)
    {
        vxReleaseTensor(&output_tensor[i]);
    }

    return status;
}

vx_status tiovx_tidl_module_release_buffers(TIOVXTIDLModuleObj *obj)
{
    vx_status status = VX_SUCCESS;

    void        *virtAddr[TIOVX_MODULES_MAX_REF_HANDLES] = {NULL};
    vx_uint32   size[TIOVX_MODULES_MAX_REF_HANDLES];
    vx_uint32   numEntries;
    vx_int32    bufq, i, ch;

    /* Free input handles */
    for(i = 0; i < obj->num_input_tensors; i++)
    {
        for(bufq = 0; bufq < obj->input[i].bufq_depth; bufq++)
        {
            for(ch = 0; ch < obj->num_cameras; ch++)
            {
                vx_reference ref = vxGetObjectArrayItem(obj->input[i].arr[bufq], ch);
                status = vxGetStatus((vx_reference)ref);

                if((vx_status)VX_SUCCESS == status)
                {
                    /* Export handles to get valid size information. */
                    status = tivxReferenceExportHandle(ref,
                                                    virtAddr,
                                                    size,
                                                    TIOVX_MODULES_MAX_REF_HANDLES,
                                                    &numEntries);

                    if((vx_status)VX_SUCCESS == status)
                    {
                        vx_int32 ctr;
                        /* Currently the vx_image buffers are alloated in one shot for multiple planes.
                            So if we are freeing this buffer then we need to get only the first plane
                            pointer address but add up the all the sizes to free the entire buffer */
                        vx_uint32 freeSize = 0;
                        for(ctr = 0; ctr < numEntries; ctr++)
                        {
                            freeSize += size[ctr];
                        }

                        if(virtAddr[0] != NULL)
                        {
                            TIOVX_MODULE_PRINTF("[TIDL-MODULE] Freeing input, bufq=%d, ch=%d, addr = 0x%016lX, size = %d \n", bufq, ch, (vx_uint64)virtAddr[0], freeSize);
                            tivxMemFree(virtAddr[0], freeSize, TIVX_MEM_EXTERNAL);
                        }

                        for(ctr = 0; ctr < numEntries; ctr++)
                        {
                            virtAddr[ctr] = NULL;
                        }

                        /* Assign NULL handles to the OpenVx objects as it will avoid
                            doing a tivxMemFree twice, once now and once during release */
                        status = tivxReferenceImportHandle(ref,
                                                        (const void **)virtAddr,
                                                        (const uint32_t *)size,
                                                        numEntries);
                    }
                    vxReleaseReference(&ref);
                }
            }
        }
    }

    /* Free output handles */
    for(i = 0; i < obj->num_output_tensors; i++)
    {
        for(bufq = 0; bufq < obj->output[i].bufq_depth; bufq++)
        {
            for(ch = 0; ch < obj->num_cameras; ch++)
            {
                vx_reference ref = vxGetObjectArrayItem(obj->output[i].arr[bufq], ch);
                status = vxGetStatus((vx_reference)ref);

                if((vx_status)VX_SUCCESS == status)
                {
                    /* Export handles to get valid size information. */
                    status = tivxReferenceExportHandle(ref,
                                                    virtAddr,
                                                    size,
                                                    TIOVX_MODULES_MAX_REF_HANDLES,
                                                    &numEntries);

                    if((vx_status)VX_SUCCESS == status)
                    {
                        vx_int32 ctr;
                        /* Currently the vx_image buffers are alloated in one shot for multiple planes.
                            So if we are freeing this buffer then we need to get only the first plane
                            pointer address but add up the all the sizes to free the entire buffer */
                        vx_uint32 freeSize = 0;
                        for(ctr = 0; ctr < numEntries; ctr++)
                        {
                            freeSize += size[ctr];
                        }

                        if(virtAddr[0] != NULL)
                        {
                            TIOVX_MODULE_PRINTF("[TIDL-MODULE] Freeing output, bufq=%d, ch=%d, addr = 0x%016lX, size = %d \n", bufq, ch, (vx_uint64)virtAddr[0], freeSize);
                            tivxMemFree(virtAddr[0], freeSize, TIVX_MEM_EXTERNAL);
                        }

                        for(ctr = 0; ctr < numEntries; ctr++)
                        {
                            virtAddr[ctr] = NULL;
                        }

                        /* Assign NULL handles to the OpenVx objects as it will avoid
                            doing a tivxMemFree twice, once now and once during release */
                        status = tivxReferenceImportHandle(ref,
                                                        (const void **)virtAddr,
                                                        (const uint32_t *)size,
                                                        numEntries);
                    }
                    vxReleaseReference(&ref);
                }
            }
        }
    }

    if ((vx_status)VX_SUCCESS != status)
    {
        VX_PRINT(VX_ZONE_ERROR, "tivxReferenceExportHandle() failed.\n");
    }

    return status;
}

vx_status tiovx_tidl_module_add_write_output_node(vx_graph graph, TIOVXTIDLModuleObj *tidlObj)
{
    vx_status status = VX_SUCCESS;

    /* Need to improve this section, currently one write node can take only one tensor. */
    vx_tensor output_tensor = (vx_tensor)vxGetObjectArrayItem(tidlObj->output[0].arr[0], 0);
    tidlObj->write_node = tivxWriteTensorNode(graph, output_tensor, tidlObj->file_path, tidlObj->file_prefix);
    vxReleaseTensor(&output_tensor);

    status = vxGetStatus((vx_reference)tidlObj->write_node);

    if((vx_status)VX_SUCCESS == status)
    {
        vxSetNodeTarget(tidlObj->write_node, VX_TARGET_STRING, TIVX_TARGET_MPU_0);

        vx_bool replicate[] = { vx_true_e, vx_false_e, vx_false_e};
        vxReplicateNode(graph, tidlObj->write_node, replicate, 3);
    }
    else
    {
        TIOVX_MODULE_ERROR("[TIDL-MODULE] Unable to create fileio write node for storing outputs! \n");
    }

    return (status);
}

vx_status tiovx_tidl_module_send_write_output_cmd(TIOVXTIDLModuleObj *tidlObj, vx_uint32 start_frame, vx_uint32 num_frames, vx_uint32 num_skip)
{
    vx_status status = VX_SUCCESS;

    tivxFileIOWriteCmd write_cmd;

    write_cmd.start_frame = start_frame;
    write_cmd.num_frames = num_frames;
    write_cmd.num_skip = num_skip;

    status = vxCopyUserDataObject(tidlObj->write_cmd, 0, sizeof(tivxFileIOWriteCmd),\
                &write_cmd, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);

    if((vx_status)VX_SUCCESS == status)
    {
        vx_reference refs[2];

        refs[0] = (vx_reference)tidlObj->write_cmd;

        status = tivxNodeSendCommand(tidlObj->write_node, TIVX_CONTROL_CMD_SEND_TO_ALL_REPLICATED_NODES,
                                TIVX_FILEIO_CMD_SET_FILE_WRITE,
                                refs, 1u);

        if(VX_SUCCESS != status)
        {
            TIOVX_MODULE_ERROR("[TIDL-MODULE] write node send command failed!\n");
        }

        TIOVX_MODULE_PRINTF("[TIDL-MODULE] write node send command success!\n");
    }

    return (status);
}

static vx_status tiovx_tidl_module_create_config(vx_context context, TIOVXTIDLModuleObj *obj)
{
    vx_status status = VX_SUCCESS;

    tivxTIDLJ7Params *params;
    vx_map_id map_id;

    if (VX_SUCCESS == status)
    {
        vxSetReferenceName((vx_reference)obj->config, "tidl_node_config");

        vxMapUserDataObject(obj->config, 0, sizeof(tivxTIDLJ7Params), &map_id,
                        (void **)&params, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST, 0);

        params->compute_config_checksum  = 0;
        params->compute_network_checksum = 0;

        memcpy(&obj->params, params, sizeof(tivxTIDLJ7Params));

        vxUnmapUserDataObject(obj->config, map_id);
    }

    return status;

}

static vx_status tiovx_tidl_module_create_network(vx_context context, TIOVXTIDLModuleObj *obj)
{
    vx_status               status = VX_SUCCESS;

    vx_map_id               map_id;
    vx_uint32               capacity;
    vx_size                 read_count;
    FILE                    *fp_network;
    void                    *network_buffer = NULL;

    fp_network = fopen(&obj->network_file_path[0], "rb");

    if(fp_network == NULL)
    {
        TIOVX_MODULE_ERROR("[TIDL-MODULE] Unable to open file! %s \n", obj->network_file_path);
        return VX_FAILURE;
    }

    fseek(fp_network, 0, SEEK_END);
    capacity = ftell(fp_network);
    fseek(fp_network, 0, SEEK_SET);

    obj->network = vxCreateUserDataObject(context, "TIDL_network", capacity, NULL );
    status = vxGetStatus((vx_reference)obj->network);

    if (VX_SUCCESS == status)
    {
        vxSetReferenceName((vx_reference)obj->network, "tidl_node_network");

        vxMapUserDataObject(obj->network, 0, capacity, &map_id,
                        (void **)&network_buffer, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST, 0);

        if(network_buffer)
        {
            read_count = fread(network_buffer, capacity, 1, fp_network);
            if(read_count != 1)
            {
                TIOVX_MODULE_ERROR("[TIDL-MODULE] Unable to read file!\n");
            }
            #ifdef COMPUTE_CHECKSUM
            sTIDL_Network_t *pNet = (sTIDL_Network_t *)network_buffer;
            if (pNet->dataFlowInfo < capacity)
            {
                uint8_t *pPerfInfo = (uint8_t *)network_buffer + pNet->dataFlowInfo;
                TIOVX_MODULE_PRINTF("[TIDL-MODULE] Computing checksum at 0x%016lX, size = %d\n", (uint64_t)pPerfInfo,  capacity - pNet->dataFlowInfo);
                getQC(pPerfInfo, &obj->network_checksum[0], capacity - pNet->dataFlowInfo);
            }
            else
            {
                TIOVX_MODULE_ERROR("[TIDL-MODULE] ERROR: pNet->dataFlowInfo should be less than %d\n", capacity);
            }
            #endif
        }
        else
        {
            TIOVX_MODULE_ERROR("[TIDL-MODULE] Unable to allocate memory for reading network! %d bytes\n", capacity);
        }
        vxUnmapUserDataObject(obj->network, map_id);
    }
    fclose(fp_network);

    return status;
}

static vx_status tiovx_tidl_module_update_checksums(TIOVXTIDLModuleObj *obj)
{
    vx_status status = VX_SUCCESS;

    tivxTIDLJ7Params *tidlParams;
    vx_map_id  map_id;

    vxMapUserDataObject(obj->config, 0, sizeof(tivxTIDLJ7Params), &map_id,
                    (void **)&tidlParams, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST, 0);

    if(tidlParams != NULL)
    {
        memcpy(tidlParams->config_checksum, &obj->config_checksum[0], TIVX_TIDL_J7_CHECKSUM_SIZE);
        memcpy(tidlParams->network_checksum, &obj->config_checksum[0], TIVX_TIDL_J7_CHECKSUM_SIZE);
    }
    else
    {
        TIOVX_MODULE_ERROR("[TIDL-MODULE] Unable to copy checksums!\n");
        status = VX_FAILURE;
    }

    vxUnmapUserDataObject(obj->config, map_id);

    return status;
}

static vx_status tiovx_tidl_module_set_createParams(vx_context context, TIOVXTIDLModuleObj *obj)
{
    vx_status status = VX_SUCCESS;
    vx_map_id  map_id;
    void *createParams_buffer = NULL;

    obj->createParams = vxCreateUserDataObject(context, "TIDL_CreateParams", sizeof(TIDL_CreateParams), NULL );
    status = vxGetStatus((vx_reference)obj->createParams);

    if(VX_SUCCESS == status)
    {
        vxSetReferenceName((vx_reference)obj->createParams, "tidl_node_createParams");

        vxMapUserDataObject(obj->createParams, 0, sizeof(TIDL_CreateParams), &map_id,
              (void **)&createParams_buffer, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST, 0);

        if(createParams_buffer)
        {
            TIDL_CreateParams *prms = createParams_buffer;
            //write create params here
            TIDL_createParamsInit(prms);

            prms->isInbufsPaded                 = 1;
            prms->quantRangeExpansionFactor     = 1.0;
            prms->quantRangeUpdateFactor        = 0.0;
            prms->traceLogLevel                 = 0;
            prms->traceWriteLevel               = 0;
        }
        else
        {
            TIOVX_MODULE_ERROR("[TIDL-MODULE] Unable to allocate memory for create time params! %ld bytes\n", sizeof(TIDL_CreateParams));
        }

        vxUnmapUserDataObject(obj->createParams, map_id);
    }

    return status;
}

static vx_status tiovx_tidl_module_create_inArgs(vx_context context, TIOVXTIDLModuleObj *obj)
{
    vx_status status = VX_SUCCESS;

    vx_user_data_object inArgs;

    inArgs = vxCreateUserDataObject(context, "TIDL_InArgs", sizeof(TIDL_InArgs), NULL );
    obj->in_args_arr  = vxCreateObjectArray(context, (vx_reference)inArgs, obj->num_cameras);
    vxReleaseUserDataObject(&inArgs);

    vxSetReferenceName((vx_reference)obj->in_args_arr, "tidl_node_in_args_arr");

    for(int i = 0; i < obj->num_cameras; i++)
    {
        vx_user_data_object inArgs;
        vx_map_id  map_id;
        void *inArgs_buffer = NULL;

        inArgs = (vx_user_data_object)vxGetObjectArrayItem(obj->in_args_arr, i);
        status = vxGetStatus((vx_reference)inArgs);

        if(VX_SUCCESS == status)
        {
            vxMapUserDataObject(inArgs, 0, sizeof(TIDL_InArgs), &map_id,
                        (void **)&inArgs_buffer, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST, 0);

            if(inArgs_buffer)
            {
                TIDL_InArgs *prms                = inArgs_buffer;
                prms->iVisionInArgs.size         = sizeof(TIDL_InArgs);
                prms->iVisionInArgs.subFrameInfo = 0;
            }
            else
            {
                TIOVX_MODULE_ERROR("[TIDL-MODULE] Unable to allocate memory for inArgs! %ld bytes\n", sizeof(TIDL_InArgs));
            }

            vxUnmapUserDataObject(inArgs, map_id);
        }

        vxReleaseUserDataObject(&inArgs);
    }

    return status;
}

static vx_status tiovx_tidl_module_create_outArgs(vx_context context, TIOVXTIDLModuleObj *obj)
{
    vx_status status = VX_SUCCESS;

    vx_user_data_object outArgs;

    outArgs = vxCreateUserDataObject(context, "TIDL_outArgs", sizeof(TIDL_outArgs), NULL );
    obj->out_args_arr  = vxCreateObjectArray(context, (vx_reference)outArgs, obj->num_cameras);
    vxReleaseUserDataObject(&outArgs);

    vxSetReferenceName((vx_reference)obj->out_args_arr, "tidl_node_out_args_arr");

    for(int i = 0; i < obj->num_cameras; i++)
    {
        vx_user_data_object outArgs;
        void *outArgs_buffer = NULL;
        vx_map_id  map_id;

        outArgs = (vx_user_data_object)vxGetObjectArrayItem(obj->out_args_arr, i);
        status = vxGetStatus((vx_reference)outArgs);

        if(VX_SUCCESS == status)
        {
            vxMapUserDataObject(outArgs, 0, sizeof(TIDL_outArgs), &map_id,
                                (void **)&outArgs_buffer, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST, 0);

            if(outArgs_buffer)
            {
                TIDL_outArgs *prms = outArgs_buffer;
                prms->iVisionOutArgs.size  = sizeof(TIDL_outArgs);
            }
            else
            {
                TIOVX_MODULE_ERROR("[TIDL-MODULE] Unable to allocate memory for outArgs! %ld bytes\n", sizeof(TIDL_outArgs));
            }

            vxUnmapUserDataObject(outArgs, map_id);
        }

        vxReleaseUserDataObject(&outArgs);
    }

    return status;
}

static vx_status tiovx_tidl_module_create_inputs(vx_context context, TIOVXTIDLModuleObj *obj)
{
    vx_status status = VX_SUCCESS;
    vx_int32 in, q;
    vx_size tensor_sizes[TIOVX_MODULES_MAX_TENSOR_DIMS];
    vx_map_id map_id_config;
    tivxTIDLJ7Params *tidlParams;
    sTIDL_IOBufDesc_t *ioBufDesc;
    vx_uint32 id;

    vxMapUserDataObject(obj->config, 0, sizeof(tivxTIDLJ7Params), &map_id_config,
                      (void **)&tidlParams, VX_READ_ONLY, VX_MEMORY_TYPE_HOST, 0);

    ioBufDesc = (sTIDL_IOBufDesc_t *)&tidlParams->ioBufDesc;
    for(id = 0; id < ioBufDesc->numInputBuf; id++) {
        vx_char name[VX_MAX_REFERENCE_NAME];

        snprintf(name, VX_MAX_REFERENCE_NAME, "tidl_node_input_tensors_%d", id);

        tensor_sizes[0] = ioBufDesc->inWidth[id]  + ioBufDesc->inPadL[id] + ioBufDesc->inPadR[id];
        tensor_sizes[1] = ioBufDesc->inHeight[id] + ioBufDesc->inPadT[id] + ioBufDesc->inPadB[id];
        tensor_sizes[2] = ioBufDesc->inNumChannels[id];

        for(in = 0; in < obj->num_input_tensors; in++)
        {
            obj->input[in].num_dims = 3;
            obj->input[in].dim_sizes[0] = tensor_sizes[0];
            obj->input[in].dim_sizes[1] = tensor_sizes[1];
            obj->input[in].dim_sizes[2] = tensor_sizes[2];
            obj->input[in].datatype = get_vx_tensor_datatype(ioBufDesc->inElementType[id]);
        }
    }

    vxUnmapUserDataObject(obj->config, map_id_config);

    for(in = 0; in < TIOVX_MODULES_MAX_TENSORS; in++)
    {
        for(q = 0; q < TIOVX_MODULES_MAX_BUFQ_DEPTH; q++)
        {
            obj->input[in].arr[q]  = NULL;
            obj->input[in].tensor_handle[q]  = NULL;
        }
    }

    for(in = 0; in < obj->num_input_tensors; in++)
    {
        vx_tensor tensor = vxCreateTensor(context, obj->input[in].num_dims, tensor_sizes, obj->input[in].datatype, 0);
        status = vxGetStatus((vx_reference)tensor);

        if((vx_status)VX_SUCCESS == status)
        {
            for(q = 0; q < obj->input[in].bufq_depth; q++)
            {
                obj->input[in].arr[q] = vxCreateObjectArray(context, (vx_reference)tensor, obj->num_cameras);
                status = vxGetStatus((vx_reference)obj->input[in].arr[q]);

                if(status != VX_SUCCESS)
                {
                    TIOVX_MODULE_ERROR("[TIDL-MODULE] Unable to create input array! \n");
                    break;
                }

                obj->input[in].tensor_handle[q] = (vx_tensor)vxGetObjectArrayItem((vx_object_array)obj->input[in].arr[q], 0);
            }
            vxReleaseTensor(&tensor);
        }
        else
        {
            TIOVX_MODULE_ERROR("[TIDL-MODULE] Unable to create input tensor template \n");
            break;
        }
    }

    return status;
}

static vx_status tiovx_tidl_module_create_outputs(vx_context context, TIOVXTIDLModuleObj *obj)
{
    vx_status status = VX_SUCCESS;
    vx_int32 in, q;
    vx_size tensor_sizes[TIOVX_MODULES_MAX_TENSOR_DIMS];
    vx_map_id map_id_config;
    tivxTIDLJ7Params *tidlParams;
    sTIDL_IOBufDesc_t *ioBufDesc;
    vx_uint32 id;

    vxMapUserDataObject(obj->config, 0, sizeof(tivxTIDLJ7Params), &map_id_config,
                      (void **)&tidlParams, VX_READ_ONLY, VX_MEMORY_TYPE_HOST, 0);

    ioBufDesc = (sTIDL_IOBufDesc_t *)&tidlParams->ioBufDesc;
    for(id = 0; id < obj->num_output_tensors; id++) {
        obj->output[id].num_dims = 3;
        obj->output[id].dim_sizes[0] = ioBufDesc->outWidth[id]  + ioBufDesc->outPadL[id] + ioBufDesc->outPadR[id];
        obj->output[id].dim_sizes[1] = ioBufDesc->outHeight[id] + ioBufDesc->outPadT[id] + ioBufDesc->outPadB[id];
        obj->output[id].dim_sizes[2] = ioBufDesc->outNumChannels[id];
        obj->output[id].datatype = get_vx_tensor_datatype(ioBufDesc->outElementType[id]);
    }

    vxUnmapUserDataObject(obj->config, map_id_config);

    for(in = 0; in < TIOVX_MODULES_MAX_TENSORS; in++)
    {
        for(q = 0; q < TIOVX_MODULES_MAX_BUFQ_DEPTH; q++)
        {
            obj->output[in].arr[q]  = NULL;
            obj->output[in].tensor_handle[q]  = NULL;
        }
    }

    for(in = 0; in < obj->num_output_tensors; in++)
    {
        tensor_sizes[0] = obj->output[in].dim_sizes[0];
        tensor_sizes[1] = obj->output[in].dim_sizes[1];
        tensor_sizes[2] = obj->output[in].dim_sizes[2];

        vx_tensor tensor = vxCreateTensor(context, obj->output[in].num_dims, tensor_sizes, obj->output[in].datatype, 0);
        status = vxGetStatus((vx_reference)tensor);

        if((vx_status)VX_SUCCESS == status)
        {
            for(q = 0; q < obj->output[in].bufq_depth; q++)
            {
                obj->output[in].arr[q] = vxCreateObjectArray(context, (vx_reference)tensor, obj->num_cameras);
                status = vxGetStatus((vx_reference)obj->output[in].arr[q]);

                if(status != VX_SUCCESS)
                {
                    TIOVX_MODULE_ERROR("[TIDL-MODULE] Unable to create output array! \n");
                    break;
                }

                obj->output[in].tensor_handle[q] = (vx_tensor)vxGetObjectArrayItem((vx_object_array)obj->output[in].arr[q], 0);
            }
            vxReleaseTensor(&tensor);
        }
        else
        {
            TIOVX_MODULE_ERROR("[TIDL-MODULE] Unable to create output tensor template");
            break;
        }
    }

    return status;
}

static void initParam(vx_reference params[], uint32_t _max_params)
{
    num_params  = 0;
    max_params = _max_params;
}

static void addParam(vx_reference params[], vx_reference obj)
{
    if(num_params <= max_params)
    {
        params[num_params] = obj;
        num_params++;
    }
    else
    {
        TIOVX_MODULE_ERROR("Error! num_params > max_params!\n");
    }  
}

#ifdef COMPUTE_CHECKSUM
static void getQC(uint8_t *pIn, uint8_t *pOut, int32_t inSize)
{
    int32_t i, j;
    uint8_t vec[TIVX_TIDL_J7_CHECKSUM_SIZE];
    int32_t remSize;

    /* Initialize vector */
    for(j = 0; j < TIVX_TIDL_J7_CHECKSUM_SIZE; j++)
    {
        vec[j] = 0;
    }

    /* Create QC */
    remSize = inSize;
    for(i = 0; i < inSize; i+=TIVX_TIDL_J7_CHECKSUM_SIZE)
    {
        int32_t elems;

        if (remSize < TIVX_TIDL_J7_CHECKSUM_SIZE)
        {
        elems = TIVX_TIDL_J7_CHECKSUM_SIZE - remSize;
        remSize += TIVX_TIDL_J7_CHECKSUM_SIZE;
        }
        else
        {
        elems = TIVX_TIDL_J7_CHECKSUM_SIZE;
        remSize -= TIVX_TIDL_J7_CHECKSUM_SIZE;
        }

        for(j = 0; j < elems; j++)
        {
        vec[j] ^= pIn[i + j];
        }
    }

    /* Return QC */
    for(j = 0; j < TIVX_TIDL_J7_CHECKSUM_SIZE; j++)
    {
        pOut[j] = vec[j];
    }
}
#endif